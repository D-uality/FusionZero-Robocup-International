import torch
import torch.nn as nn
import torchvision.transforms as transforms
from PIL import Image
import numpy as np
import cv2
import time

class CustomSilverDetector(nn.Module):
    def __init__(self, input_size=64, class_count=2):
        super(CustomSilverDetector, self).__init__()

        self.features = nn.Sequential(
            nn.Conv2d(3, 16, kernel_size=3, stride=2, padding=1, bias=False),
            nn.BatchNorm2d(16),
            nn.ReLU(inplace=True),
            nn.Conv2d(16, 32, kernel_size=3, stride=2, padding=1, bias=False),
            nn.BatchNorm2d(32),
            nn.ReLU(inplace=True),
            nn.Conv2d(32, 48, kernel_size=3, stride=2, padding=1, bias=False),
            nn.BatchNorm2d(48),
            nn.ReLU(inplace=True),
            nn.Conv2d(48, 64, kernel_size=3, stride=2, padding=1, bias=False),
            nn.BatchNorm2d(64),
            nn.ReLU(inplace=True),
        )

        self.classifier = nn.Sequential(
            nn.AdaptiveAvgPool2d(1),
            nn.Flatten(),
            nn.Linear(64, 32),
            nn.ReLU(inplace=True),
            nn.Linear(32, class_count)
        )

    def forward(self, x):
        x = self.features(x)
        x = self.classifier(x)
        return x

class SilverLineDetector:
    def __init__(self, model_path, device='cpu'):
        self.device = torch.device(device)
        self.model = self._load_model(model_path)
        self.transform = transforms.Compose([
            transforms.Resize((64, 64)),
            transforms.ToTensor(),
            transforms.Normalize(mean=[0.485, 0.456, 0.406],
                                 std=[0.229, 0.224, 0.225])
        ])
        self.class_names = ["No Silver", "Silver"]
        self._warmup()

    def _load_model(self, model_path):
        try:
            print("Trying TorchScript load...")
            model = torch.jit.load(model_path, map_location=self.device)
            print("✓ Loaded TorchScript model")
            return model
        except Exception as e:
            print(f"TorchScript failed: {e}")
            try:
                print("Trying PyTorch state dict...")
                model = CustomSilverDetector()
                model.load_state_dict(torch.load(model_path, map_location=self.device))
                model.to(self.device)
                model.eval()
                print("✓ Loaded PyTorch model")
                return model
            except Exception as e2:
                print(f"State dict failed: {e2}")
                print("Trying full model load...")
                model = torch.load(model_path, map_location=self.device)
                model.to(self.device)
                model.eval()
                print("✓ Loaded full PyTorch model")
                return model

    def _warmup(self):
        with torch.no_grad():
            dummy = torch.randn(1, 3, 64, 64).to(self.device)
            self.model(dummy)

    def preprocess_image(self, image):
        if isinstance(image, np.ndarray):
            if len(image.shape) == 3 and image.shape[2] == 3:
                image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            image = Image.fromarray(image)
        return self.transform(image).unsqueeze(0).to(self.device)

    def predict(self, image, return_probabilities=False):
        image_tensor = self.preprocess_image(image)
        with torch.no_grad():
            outputs = self.model(image_tensor)
            probs = torch.nn.functional.softmax(outputs, dim=1)
            pred = torch.argmax(outputs, dim=1).item()
            confidence = probs[0][pred].item()
        result = {
            'prediction': pred,
            'class_name': self.class_names[pred],
            'confidence': confidence
        }
        if return_probabilities:
            result['probabilities'] = {
                'no_silver': probs[0][0].item(),
                'silver': probs[0][1].item()
            }
        return result