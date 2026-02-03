import torch
import torch.nn as nn
import torch.nn.functional as F

class PilotNet(nn.Module):
    def __init__(self, image_shape=(120, 160, 3), dropout=0.0):
        super().__init__()
        H, W, C = image_shape

        # BatchNorm en la entrada (igual que entrenamiento)
        self.bn_in = nn.BatchNorm2d(C, eps=1e-3)

        # Convs estilo PilotNet
        self.cn_1 = nn.Conv2d(C, 24, kernel_size=5, stride=2)
        self.cn_2 = nn.Conv2d(24, 36, kernel_size=5, stride=2)
        self.cn_3 = nn.Conv2d(36, 48, kernel_size=5, stride=2)
        self.cn_4 = nn.Conv2d(48, 64, kernel_size=3, stride=1)
        self.cn_5 = nn.Conv2d(64, 64, kernel_size=3, stride=1)

        self.flatten = nn.Flatten()

        # Calcular automáticamente in_features
        with torch.no_grad():
            dummy = torch.zeros(1, C, H, W)
            z = self._forward_conv(dummy)
            in_features = z.numel()

        # Tronco denso
        self.fc_1 = nn.Linear(in_features, 1164)
        self.fc_2 = nn.Linear(1164, 100)
        self.fc_3 = nn.Linear(100, 50)
        self.fc_4 = nn.Linear(50, 10)

        self.drop = nn.Dropout(dropout)

        # Cabezas separadas
        self.head_steer = nn.Linear(10, 1)
        self.head_throttle = nn.Linear(10, 1)

    def _forward_conv(self, x):
        x = self.bn_in(x)
        x = F.relu(self.cn_1(x))
        x = F.relu(self.cn_2(x))
        x = F.relu(self.cn_3(x))
        x = F.relu(self.cn_4(x))
        x = F.relu(self.cn_5(x))
        x = self.flatten(x)
        return x

    def forward(self, x):
        x = self._forward_conv(x)

        x = F.relu(self.fc_1(x))
        x = self.drop(x)
        x = F.relu(self.fc_2(x))
        x = self.drop(x)
        x = F.relu(self.fc_3(x))
        x = F.relu(self.fc_4(x))

        # Activaciones correctas
        steer = torch.tanh(self.head_steer(x))          # [-1,1]
        throttle = torch.sigmoid(self.head_throttle(x)) # [0,1]

        return torch.cat([steer, throttle], dim=1)      # (B,2)

def load_pilotnet(model_path, device="cpu"):
    checkpoint = torch.load(model_path, map_location=device)

    model = PilotNet(image_shape=(120, 160, 3), dropout=0.0).to(device)
    model.load_state_dict(checkpoint["model_state_dict"])
    model.eval()
    return model