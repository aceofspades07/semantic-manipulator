"""
Text Classifier for Robot Actions using model2vec
Classifies user prompts into: pick (with color), place, or drop
"""

import numpy as np
from sklearn.linear_model import LogisticRegression
from sklearn.preprocessing import LabelEncoder
import pickle
import re

# Install required packages first:
# pip install model2vec scikit-learn --break-system-packages

try:
    from model2vec import StaticModel
except ImportError:
    print("Installing model2vec...")
    import subprocess
    subprocess.run(["pip", "install", "model2vec", "--break-system-packages"])
    from model2vec import StaticModel


class ActionClassifier:
    def __init__(self):
        """Initialize the classifier with model2vec embeddings"""
        print("Loading model2vec model...")
        # Using a small, fast model
        self.embedding_model = StaticModel.from_pretrained("minishlab/potion-base-8M")
        self.classifier = LogisticRegression(max_iter=1000, random_state=42)
        self.label_encoder = LabelEncoder()
        self.color_pattern = re.compile(r'\b(red|blue|green|yellow|orange|purple|black|white|pink|brown|gray|grey)\b', re.IGNORECASE)
        
    def prepare_training_data(self):
        """Create training dataset with various phrasings"""
        training_data = [
            # Pick commands with colors
            ("pick the red block", "pick"),
            ("pick up the blue object", "pick"),
            ("grab the green cube", "pick"),
            ("take the yellow item", "pick"),
            ("get the red one", "pick"),
            ("pick red", "pick"),
            ("pick up red block", "pick"),
            ("grasp the blue sphere", "pick"),
            ("hold the green object", "pick"),
            ("fetch the orange block", "pick"),
            ("retrieve the purple item", "pick"),
            ("pick the white cube", "pick"),
            ("take red", "pick"),
            ("grab blue object", "pick"),
            ("pick up the black block", "pick"),
            
            # Place commands
            ("place it here", "place"),
            ("place the block", "place"),
            ("put it down", "place"),
            ("set it on the table", "place"),
            ("place on the platform", "place"),
            ("put the object here", "place"),
            ("place it there", "place"),
            ("set it down", "place"),
            ("position it here", "place"),
            ("place the item", "place"),
            ("put it on the shelf", "place"),
            ("place object", "place"),
            ("set the block down", "place"),
            ("put down", "place"),
            ("place here", "place"),
            
            # Drop commands
            ("drop it", "drop"),
            ("drop the block", "drop"),
            ("release it", "drop"),
            ("let go", "drop"),
            ("drop the object", "drop"),
            ("release the item", "drop"),
            ("drop now", "drop"),
            ("let it go", "drop"),
            ("release", "drop"),
            ("drop", "drop"),
            ("let go of it", "drop"),
            ("drop the cube", "drop"),
            ("release object", "drop"),
            ("drop it now", "drop"),
            ("let go of the block", "drop"),

            ("stop", "none"),
            ("halt", "none"),
            ("freeze", "none"),
            ("emergency stop", "none"),
            ("pause execution", "none"),
            ("shut down", "none"),
            ("restart", "none"),
            ("abort mission", "none"),
            ("cancel", "none"),
            ("wait", "none"),

            ("push the block", "none"),
            ("nudge it", "none"),
            ("slide the cube", "none"),
            ("throw the object", "none"),
            ("stack the items", "none"), 
            ("shake the bottle", "none"),
            ("flip it over", "none"),
            ("unscrew the cap", "none"),

            ("what is your battery level", "none"),
            ("are you connected?", "none"),
            ("what do you see?", "none"),
            ("identify the object", "none"),
            ("where is the blue block?", "none"),
            ("how many cubes are there?", "none"),
            ("scan the area", "none"),
            ("check status", "none"),
            ("is the gripper open?", "none"),
            ("who created you?", "none")
        ]
        
        return training_data
    
    def extract_color(self, text):
        """Extract color parameter from text"""
        match = self.color_pattern.search(text)
        return match.group(1).lower() if match else None
    
    def train(self):
        """Train the classifier"""
        print("Preparing training data...")
        training_data = self.prepare_training_data()
        
        texts = [item[0] for item in training_data]
        labels = [item[1] for item in training_data]
        
        print("Generating embeddings...")
        # Get embeddings for all training texts
        embeddings = self.embedding_model.encode(texts)
        
        print("Training classifier...")
        # Encode labels
        encoded_labels = self.label_encoder.fit_transform(labels)
        
        # Train classifier
        self.classifier.fit(embeddings, encoded_labels)
        
        print("Training complete!")
        print(f"Classes: {self.label_encoder.classes_}")
        
    def predict(self, text):
        """
        Classify a user prompt
        Returns: (action, color) where color is None for place/drop actions
        """
        # Get embedding
        embedding = self.embedding_model.encode([text])
        
        # Predict action
        prediction = self.classifier.predict(embedding)[0]
        action = self.label_encoder.inverse_transform([prediction])[0]
        
        # Extract color if it's a pick action
        color = None
        if action == "pick":
            color = self.extract_color(text)
        
        # Get confidence scores
        probabilities = self.classifier.predict_proba(embedding)[0]
        confidence = max(probabilities)
        
        return {
            'action': action,
            'color': color,
            'confidence': confidence,
            'all_probabilities': {
                label: prob 
                for label, prob in zip(self.label_encoder.classes_, probabilities)
            }
        }
    
    def save(self, filepath='action_classifier.pkl'):
        """Save the trained classifier"""
        with open(filepath, 'wb') as f:
            pickle.dump({
                'classifier': self.classifier,
                'label_encoder': self.label_encoder
            }, f)
        print(f"Classifier saved to {filepath}")
    
    def load(self, filepath='action_classifier.pkl'):
        """Load a trained classifier"""
        with open(filepath, 'rb') as f:
            data = pickle.load(f)
            self.classifier = data['classifier']
            self.label_encoder = data['label_encoder']
        print(f"Classifier loaded from {filepath}")


def main():
    """Demo of the classifier"""
    print("=" * 60)
    print("Action Classifier for Robot Commands")
    print("=" * 60)
    
    # Initialize and train
    classifier = ActionClassifier()
    classifier.train()
    
    print("\n" + "=" * 60)
    print("Testing the classifier:")
    print("=" * 60)
    
    # Test examples
    test_prompts = [
        "pick the red block",
        "grab the blue cube",
        "place it on the table",
        "drop the object",
        "take the green one",
        "put it down here",
        "release it",
        "pick up the yellow sphere",
        "set the block on the platform",
        "let go of it"
    ]
    
    for prompt in test_prompts:
        result = classifier.predict(prompt)
        print(f"\nPrompt: '{prompt}'")
        print(f"  → Action: {result['action']}")
        if result['color']:
            print(f"  → Color: {result['color']}")
        print(f"  → Confidence: {result['confidence']:.2%}")
    
    # Save the model
    print("\n" + "=" * 60)
    classifier.save()
    
    # Interactive mode
    print("\n" + "=" * 60)
    print("Interactive Mode (type 'quit' to exit)")
    print("=" * 60)
    
    while True:
        user_input = input("\nEnter command: ").strip()
        if user_input.lower() in ['quit', 'exit', 'q']:
            break
        if not user_input:
            continue
            
        result = classifier.predict(user_input)
        print(f"  → Action: {result['action']}")
        if result['color']:
            print(f"  → Color: {result['color']}")
        print(f"  → Confidence: {result['confidence']:.2%}")


if __name__ == "__main__":
    main()