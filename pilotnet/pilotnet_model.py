#!/usr/bin/env python3

import torch.nn as nn


class PilotNet(nn.Module):
    """Minimal PilotNet-style steering regressor."""

    def __init__(self, in_channels=3, dropout=0.1):
        super().__init__()
        self.features = nn.Sequential(
            nn.Conv2d(in_channels, 24, kernel_size=5, stride=2),
            nn.ReLU(inplace=True),
            nn.Conv2d(24, 36, kernel_size=5, stride=2),
            nn.ReLU(inplace=True),
            nn.Conv2d(36, 48, kernel_size=5, stride=2),
            nn.ReLU(inplace=True),
            nn.Conv2d(48, 64, kernel_size=3, stride=1),
            nn.ReLU(inplace=True),
            nn.Conv2d(64, 64, kernel_size=3, stride=1),
            nn.ReLU(inplace=True),
            nn.AdaptiveAvgPool2d((1, 1)),
        )
        self.regressor = nn.Sequential(
            nn.Flatten(),
            nn.Linear(64, 100),
            nn.ReLU(inplace=True),
            nn.Dropout(p=dropout),
            nn.Linear(100, 50),
            nn.ReLU(inplace=True),
            nn.Dropout(p=dropout),
            nn.Linear(50, 10),
            nn.ReLU(inplace=True),
            nn.Linear(10, 1),
        )

    def forward(self, x):
        x = self.features(x)
        x = self.regressor(x)
        return x.squeeze(-1)
