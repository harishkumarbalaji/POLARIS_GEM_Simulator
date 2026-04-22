#!/usr/bin/env python3

import argparse
import csv
import glob
import json
import math
import os
import random
from dataclasses import dataclass

import numpy as np
from PIL import Image
import torch
import torch.nn.functional as F
from torch.utils.data import DataLoader, Dataset

from pilotnet_model import PilotNet


@dataclass
class Sample:
    image_path: str
    steering: float
    speed: float
    x: float
    y: float
    yaw: float
    session: str


class PilotNetDataset(Dataset):
    def __init__(self, samples, image_width, image_height, crop_top_ratio, crop_bottom_ratio):
        self.samples = samples
        self.image_width = image_width
        self.image_height = image_height
        self.crop_top_ratio = crop_top_ratio
        self.crop_bottom_ratio = crop_bottom_ratio

    def __len__(self):
        return len(self.samples)

    def __getitem__(self, index):
        sample = self.samples[index]
        image = Image.open(sample.image_path).convert("RGB")
        image = self.preprocess_image(image)
        steering = torch.tensor(sample.steering, dtype=torch.float32)
        return image, steering

    def preprocess_image(self, image):
        width, height = image.size
        top = int(height * self.crop_top_ratio)
        bottom = int(height * (1.0 - self.crop_bottom_ratio))
        bottom = max(bottom, top + 1)
        image = image.crop((0, top, width, bottom))
        image = image.resize((self.image_width, self.image_height), Image.BILINEAR)
        image = np.asarray(image, dtype=np.float32) / 255.0
        image = np.transpose(image, (2, 0, 1))
        return torch.from_numpy(image)


def parse_args():
    parser = argparse.ArgumentParser(description="Train a PilotNet-style steering regressor")
    parser.add_argument(
        "--data-root",
        default=os.path.expanduser("~/pilotnet_data"),
        help="Root containing session directories with metadata.csv and images/",
    )
    parser.add_argument(
        "--metadata-glob",
        default="**/metadata.csv",
        help="Glob under --data-root used to discover metadata files",
    )
    parser.add_argument("--output-dir", default="pilotnet_runs/run_001", help="Training output directory")
    parser.add_argument("--epochs", type=int, default=15)
    parser.add_argument("--batch-size", type=int, default=64)
    parser.add_argument("--learning-rate", type=float, default=1e-3)
    parser.add_argument("--weight-decay", type=float, default=1e-4)
    parser.add_argument("--val-fraction", type=float, default=0.2)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--image-width", type=int, default=200)
    parser.add_argument("--image-height", type=int, default=66)
    parser.add_argument("--crop-top-ratio", type=float, default=0.35)
    parser.add_argument("--crop-bottom-ratio", type=float, default=0.10)
    parser.add_argument("--num-workers", type=int, default=4)
    parser.add_argument("--dropout", type=float, default=0.1)
    parser.add_argument("--loss", choices=["mse", "smooth_l1"], default="smooth_l1")
    parser.add_argument(
        "--min-abs-speed",
        type=float,
        default=0.0,
        help="Ignore samples below this absolute speed",
    )
    parser.add_argument(
        "--max-samples-per-session",
        type=int,
        default=0,
        help="If >0, cap samples per session for quick experiments",
    )
    return parser.parse_args()


def set_seed(seed):
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    torch.cuda.manual_seed_all(seed)


def discover_metadata_files(data_root, metadata_glob):
    pattern = os.path.join(data_root, metadata_glob)
    files = sorted(glob.glob(pattern, recursive=True))
    if not files:
        raise RuntimeError(f"No metadata files found under {data_root} with glob {metadata_glob}")
    return files


def load_samples(metadata_files, min_abs_speed=0.0, max_samples_per_session=0):
    sessions = {}
    for metadata_path in metadata_files:
        session_dir = os.path.dirname(metadata_path)
        session_name = os.path.basename(session_dir)
        session_samples = []
        with open(metadata_path, newline="") as csvfile:
            reader = csv.DictReader(csvfile)
            for row in reader:
                speed = float(row["speed"])
                if abs(speed) < min_abs_speed:
                    continue
                image_path = os.path.join(session_dir, row["image_path"])
                if not os.path.exists(image_path):
                    continue
                session_samples.append(
                    Sample(
                        image_path=image_path,
                        steering=float(row["steering"]),
                        speed=speed,
                        x=float(row["x"]),
                        y=float(row["y"]),
                        yaw=float(row["yaw"]),
                        session=session_name,
                    )
                )
        if max_samples_per_session > 0:
            session_samples = session_samples[:max_samples_per_session]
        if session_samples:
            sessions[session_name] = session_samples

    if not sessions:
        raise RuntimeError("No usable samples found in discovered metadata files")
    return sessions


def split_single_session(samples, val_fraction):
    total = len(samples)
    if total < 2:
        raise RuntimeError("Need at least 2 samples to split a single-session dataset")

    val_count = max(1, int(math.ceil(total * val_fraction)))
    if val_count >= total:
        val_count = total - 1

    split_index = total - val_count
    train_samples = samples[:split_index]
    val_samples = samples[split_index:]
    return train_samples, val_samples


def split_sessions(session_samples, val_fraction, seed):
    session_names = sorted(session_samples.keys())
    if len(session_names) == 1:
        session_name = session_names[0]
        train_samples, val_samples = split_single_session(session_samples[session_name], val_fraction)
        return train_samples, val_samples, [session_name], [session_name], "single_session_frame_split"

    rng = random.Random(seed)
    rng.shuffle(session_names)
    val_count = max(1, int(math.ceil(len(session_names) * val_fraction)))
    if val_count >= len(session_names) and len(session_names) > 1:
        val_count = len(session_names) - 1

    val_sessions = set(session_names[:val_count])
    train_sessions = [name for name in session_names if name not in val_sessions]
    if not train_sessions:
        raise RuntimeError("Need at least one training session after split")

    train_samples = []
    val_samples = []
    for session_name, samples in session_samples.items():
        if session_name in val_sessions:
            val_samples.extend(samples)
        else:
            train_samples.extend(samples)
    return train_samples, val_samples, train_sessions, sorted(val_sessions), "session_split"


def build_loaders(args, train_samples, val_samples):
    train_dataset = PilotNetDataset(
        train_samples,
        args.image_width,
        args.image_height,
        args.crop_top_ratio,
        args.crop_bottom_ratio,
    )
    val_dataset = PilotNetDataset(
        val_samples,
        args.image_width,
        args.image_height,
        args.crop_top_ratio,
        args.crop_bottom_ratio,
    )
    train_loader = DataLoader(
        train_dataset,
        batch_size=args.batch_size,
        shuffle=True,
        num_workers=args.num_workers,
        pin_memory=torch.cuda.is_available(),
    )
    val_loader = DataLoader(
        val_dataset,
        batch_size=args.batch_size,
        shuffle=False,
        num_workers=args.num_workers,
        pin_memory=torch.cuda.is_available(),
    )
    return train_loader, val_loader


def compute_loss(predictions, targets, loss_name):
    if loss_name == "mse":
        return F.mse_loss(predictions, targets)
    return F.smooth_l1_loss(predictions, targets)


def run_epoch(model, loader, optimizer, device, loss_name, training):
    if training:
        model.train()
    else:
        model.eval()

    running_loss = 0.0
    running_mae = 0.0
    total = 0

    for images, steering in loader:
        images = images.to(device)
        steering = steering.to(device)

        if training:
            optimizer.zero_grad(set_to_none=True)

        with torch.set_grad_enabled(training):
            predictions = model(images)
            loss = compute_loss(predictions, steering, loss_name)

        if training:
            loss.backward()
            optimizer.step()

        batch_size = images.shape[0]
        running_loss += float(loss.item()) * batch_size
        running_mae += float(torch.abs(predictions - steering).mean().item()) * batch_size
        total += batch_size

    return {
        "loss": running_loss / max(total, 1),
        "mae": running_mae / max(total, 1),
    }


def save_json(path, payload):
    with open(path, "w") as handle:
        json.dump(payload, handle, indent=2)


def main():
    args = parse_args()
    set_seed(args.seed)

    metadata_files = discover_metadata_files(args.data_root, args.metadata_glob)
    session_samples = load_samples(
        metadata_files,
        min_abs_speed=args.min_abs_speed,
        max_samples_per_session=args.max_samples_per_session,
    )
    train_samples, val_samples, train_sessions, val_sessions, split_mode = split_sessions(
        session_samples,
        args.val_fraction,
        args.seed,
    )

    os.makedirs(args.output_dir, exist_ok=True)
    train_loader, val_loader = build_loaders(args, train_samples, val_samples)

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    model = PilotNet(dropout=args.dropout).to(device)
    optimizer = torch.optim.Adam(
        model.parameters(),
        lr=args.learning_rate,
        weight_decay=args.weight_decay,
    )

    metrics = {
        "split_mode": split_mode,
        "train_sessions": train_sessions,
        "val_sessions": val_sessions,
        "num_train_samples": len(train_samples),
        "num_val_samples": len(val_samples),
        "epochs": [],
    }

    if split_mode != "session_split":
        print(
            "warning: only one session was found, so validation uses a frame split "
            "within the same session and may be optimistic"
        )

    best_val_loss = float("inf")
    best_ckpt_path = os.path.join(args.output_dir, "best_model.pt")
    latest_ckpt_path = os.path.join(args.output_dir, "latest_model.pt")

    for epoch in range(1, args.epochs + 1):
        train_metrics = run_epoch(model, train_loader, optimizer, device, args.loss, training=True)
        val_metrics = run_epoch(model, val_loader, optimizer, device, args.loss, training=False)

        epoch_metrics = {
            "epoch": epoch,
            "train_loss": train_metrics["loss"],
            "train_mae": train_metrics["mae"],
            "val_loss": val_metrics["loss"],
            "val_mae": val_metrics["mae"],
        }
        metrics["epochs"].append(epoch_metrics)

        checkpoint = {
            "model_state_dict": model.state_dict(),
            "args": vars(args),
            "epoch": epoch,
            "metrics": epoch_metrics,
        }
        torch.save(checkpoint, latest_ckpt_path)

        if val_metrics["loss"] < best_val_loss:
            best_val_loss = val_metrics["loss"]
            torch.save(checkpoint, best_ckpt_path)

        print(
            f"epoch={epoch:03d} "
            f"train_loss={train_metrics['loss']:.6f} train_mae={train_metrics['mae']:.6f} "
            f"val_loss={val_metrics['loss']:.6f} val_mae={val_metrics['mae']:.6f}"
        )

    save_json(os.path.join(args.output_dir, "metrics.json"), metrics)
    save_json(os.path.join(args.output_dir, "train_args.json"), vars(args))
    print(f"saved best checkpoint to {best_ckpt_path}")


if __name__ == "__main__":
    main()
