#!/usr/bin/env python3
"""
Raspberry Pi Silver Line Detection Inference Script
"""

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

        """
        - 64 -> 32 -> 16 ->  8 ->  4 spatial resolution: forces features to group into heirarchial structure
        -  3 -> 16 -> 32 -> 48 -> 64 feature maps: increases detail and quality for better strucutre identification
        - All use ReLU activation function to break linearity
        -
        """
        self.features = nn.Sequential(
            # Block 1: 64x64 -> 32x32
            nn.Conv2d(3, 16, kernel_size=3, stride=2, padding=1, bias=False),
            nn.BatchNorm2d(16),
            nn.ReLU(inplace=True),

            # Block 2: 32x32 -> 16x16
            nn.Conv2d(16, 32, kernel_size=3, stride=2, padding=1, bias=False),
            nn.BatchNorm2d(32),
            nn.ReLU(inplace=True),

            # Block 3: 16x16 -> 8x8
            nn.Conv2d(32, 48, kernel_size=3, stride=2, padding=1, bias=False),
            nn.BatchNorm2d(48),
            nn.ReLU(inplace=True),

            # Block 4: 8x8 -> 4x4
            nn.Conv2d(48, 64, kernel_size=3, stride=2, padding=1, bias=False),
            nn.BatchNorm2d(64),
            nn.ReLU(inplace=True),
        )

        self.classifier = nn.Sequential(
            nn.AdaptiveAvgPool2d(1),
            nn.Flatten(),
            nn.Dropout(0.3),
            nn.Linear(64, 32),
            nn.ReLU(inplace=True),
            nn.Dropout(0.2),
            nn.Linear(32, class_count)
        )

    def forward(self, x):
        x = self.features(x)
        x = self.classifier(x)

        return x

class SilverLineDetector:
    def __init__(self, model_path, device='cpu'):
        """
        Initialize the Silver Line Detector
        
        Args:
            model_path: Path to the trained model (.pt or .pth file)
            device: Device to run inference on ('cpu' or 'cuda')
        """
        self.device = torch.device(device)
        self.model = self._load_model(model_path)
        self.transform = self._get_transform()
        self.class_names = ["No Silver", "Silver"]
        
        # Warm up the model
        self._warmup()
        
    def _load_model(self, model_path):
        """Load the trained model"""
        try:
            # Try loading as TorchScript first
            print("Attempting to load TorchScript model...")
            model = torch.jit.load(model_path, map_location=self.device)
            print("✓ TorchScript model loaded successfully")
            return model
            
        except Exception as e:
            print(f"TorchScript loading failed: {e}")
            print("Attempting to load as PyTorch state dict...")
            
            try:
                # Try loading as state dict
                model = CustomSilverDetector(input_size=64, class_count=2)
                model.load_state_dict(torch.load(model_path, map_location=self.device))
                model.to(self.device)
                model.eval()
                print("✓ PyTorch state dict loaded successfully")
                return model
                
            except Exception as e2:
                print(f"State dict loading failed: {e2}")
                print("Attempting to load complete model...")
                
                try:
                    # Try loading complete model
                    model = torch.load(model_path, map_location=self.device)
                    model.to(self.device)
                    model.eval()
                    print("✓ Complete model loaded successfully")
                    return model
                    
                except Exception as e3:
                    raise Exception(f"Failed to load model with all methods: {e3}")
    
    def _get_transform(self):
        """Get image preprocessing transform"""
        return transforms.Compose([
            transforms.Resize((64, 64)),
            transforms.ToTensor(),
            transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
        ])
    
    def _warmup(self):
        """Warm up the model with dummy input"""
        dummy_input = torch.randn(1, 3, 64, 64).to(self.device)
        with torch.no_grad():
            _ = self.model(dummy_input)
        print("✓ Model warmed up")
    
    def preprocess_image(self, image):
        """
        Preprocess image for inference
        
        Args:
            image: Input image (numpy array, PIL Image, or OpenCV image)
            
        Returns:
            torch.Tensor: Preprocessed image tensor
        """
        if isinstance(image, np.ndarray):
            # Convert from OpenCV (BGR) to PIL (RGB)
            if len(image.shape) == 3 and image.shape[2] == 3:
                image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            image = Image.fromarray(image)
        
        # Apply transforms
        image_tensor = self.transform(image).unsqueeze(0).to(self.device)
        return image_tensor
    
    def predict(self, image, return_probabilities=False):
        """
        Predict silver line presence in image
        
        Args:
            image: Input image
            return_probabilities: Whether to return class probabilities
            
        Returns:
            dict: Prediction results
        """
        start_time = time.time()
        
        # Preprocess
        image_tensor = self.preprocess_image(image)
        
        # Inference
        with torch.no_grad():
            outputs = self.model(image_tensor)
            probabilities = torch.nn.functional.softmax(outputs, dim=1)
            prediction = torch.argmax(outputs, dim=1).item()
            confidence = probabilities[0][prediction].item()
        
        inference_time = time.time() - start_time
        
        result = {
            'prediction': prediction,
            'class_name': self.class_names[prediction],
            'confidence': confidence,
            'inference_time': inference_time
        }
        
        if return_probabilities:
            result['probabilities'] = {
                'no_silver': probabilities[0][0].item(),
                'silver': probabilities[0][1].item()
            }
        
        return result
    
    def predict_batch(self, images):
        """
        Predict on a batch of images
        
        Args:
            images: List of images
            
        Returns:
            list: List of prediction results
        """
        batch_tensor = torch.stack([self.preprocess_image(img).squeeze(0) for img in images]).to(self.device)
        
        with torch.no_grad():
            outputs = self.model(batch_tensor)
            probabilities = torch.nn.functional.softmax(outputs, dim=1)
            predictions = torch.argmax(outputs, dim=1)
        
        results = []
        for i in range(len(images)):
            result = {
                'prediction': predictions[i].item(),
                'class_name': self.class_names[predictions[i].item()],
                'confidence': probabilities[i][predictions[i]].item()
            }
            results.append(result)
        
        return results