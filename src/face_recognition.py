#!/usr/bin/env python3
"""
Simple Face Recognition Module for Booster K1
Uses DeepFace for face recognition with fallback to basic detection
"""

import cv2
import numpy as np
import os
import json
from datetime import datetime
from typing import Dict, List, Tuple, Optional

class FaceRecognizer:
    """Simple face recognition using DeepFace"""

    def __init__(self, database_path='face_database.json', use_deepface=True):
        self.database_path = database_path
        self.database = self._load_database()
        self.use_deepface = use_deepface
        self.deepface_available = False

        # Try to import DeepFace
        if use_deepface:
            try:
                from deepface import DeepFace
                self.DeepFace = DeepFace
                self.deepface_available = True
                print("✓ DeepFace loaded successfully")
                print(f"  Using model: VGG-Face")
                print(f"  Distance metric: cosine")
            except ImportError:
                print("⚠️  DeepFace not available. Install with: pip install deepface")
                print("   Falling back to basic face detection only")
                self.use_deepface = False

        # Always load OpenCV face detector for face detection
        cascade_path = cv2.data.haarcascades + 'haarcascade_frontalface_default.xml'
        self.face_cascade = cv2.CascadeClassifier(cascade_path)

        if self.face_cascade.empty():
            print("✗ Failed to load face cascade classifier")
        else:
            print("✓ OpenCV face detector loaded")

    def _load_database(self) -> Dict:
        """Load face database from JSON file"""
        if os.path.exists(self.database_path):
            try:
                with open(self.database_path, 'r') as f:
                    db = json.load(f)
                    print(f"✓ Loaded database with {len(db.get('people', []))} people")
                    return db
            except Exception as e:
                print(f"⚠️  Error loading database: {e}")
                return {'people': [], 'version': '1.0'}
        else:
            print("Creating new face database")
            return {'people': [], 'version': '1.0'}

    def _save_database(self):
        """Save face database to JSON file"""
        try:
            with open(self.database_path, 'w') as f:
                json.dump(self.database, f, indent=2)
            print(f"✓ Database saved ({len(self.database['people'])} people)")
        except Exception as e:
            print(f"✗ Error saving database: {e}")

    def detect_faces(self, frame: np.ndarray) -> List[Tuple[int, int, int, int]]:
        """
        Detect faces in frame using OpenCV
        Returns list of (x, y, w, h) tuples
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        faces = self.face_cascade.detectMultiScale(
            gray,
            scaleFactor=1.3,
            minNeighbors=5,
            minSize=(30, 30)
        )
        return faces

    def get_face_embedding(self, face_img: np.ndarray) -> Optional[List[float]]:
        """
        Get face embedding using DeepFace
        Returns embedding vector or None if failed
        """
        if not self.deepface_available:
            return None

        try:
            # DeepFace expects BGR images (OpenCV format)
            embedding_objs = self.DeepFace.represent(
                img_path=face_img,
                model_name='VGG-Face',
                enforce_detection=False
            )

            if embedding_objs and len(embedding_objs) > 0:
                return embedding_objs[0]['embedding']
            return None

        except Exception as e:
            print(f"⚠️  Error getting embedding: {e}")
            return None

    def compare_embeddings(self, emb1: List[float], emb2: List[float]) -> float:
        """
        Compare two embeddings using cosine similarity
        Returns similarity score (0-1, higher is more similar)
        """
        emb1 = np.array(emb1)
        emb2 = np.array(emb2)

        # Cosine similarity
        dot_product = np.dot(emb1, emb2)
        norm1 = np.linalg.norm(emb1)
        norm2 = np.linalg.norm(emb2)

        if norm1 == 0 or norm2 == 0:
            return 0.0

        similarity = dot_product / (norm1 * norm2)
        return float(similarity)

    def recognize_face(self, face_img: np.ndarray, threshold: float = 0.6) -> Tuple[Optional[str], float]:
        """
        Recognize a face by comparing to database
        Returns (name, confidence) or (None, 0.0) if unknown
        threshold: minimum similarity score to consider a match
        """
        if not self.deepface_available:
            return None, 0.0

        # Get embedding for the face
        embedding = self.get_face_embedding(face_img)
        if embedding is None:
            return None, 0.0

        # Compare with all known faces
        best_match = None
        best_similarity = 0.0

        for person in self.database['people']:
            if 'embeddings' not in person or len(person['embeddings']) == 0:
                continue

            # Compare with all embeddings for this person (take best match)
            for stored_embedding in person['embeddings']:
                similarity = self.compare_embeddings(embedding, stored_embedding)

                if similarity > best_similarity:
                    best_similarity = similarity
                    best_match = person['name']

        # Check if best match exceeds threshold
        if best_similarity >= threshold:
            return best_match, best_similarity
        else:
            return None, best_similarity

    def add_person(self, name: str, face_img: np.ndarray) -> bool:
        """
        Add a new person to the database
        Returns True if successful
        """
        if not self.deepface_available:
            print("⚠️  DeepFace not available, cannot add person")
            return False

        # Get embedding
        embedding = self.get_face_embedding(face_img)
        if embedding is None:
            print(f"✗ Failed to get embedding for {name}")
            return False

        # Check if person already exists
        for person in self.database['people']:
            if person['name'].lower() == name.lower():
                # Add embedding to existing person
                person['embeddings'].append(embedding)
                person['last_seen'] = datetime.now().isoformat()
                person['times_seen'] = person.get('times_seen', 0) + 1
                self._save_database()
                print(f"✓ Added new embedding for {name} ({len(person['embeddings'])} total)")
                return True

        # Add new person
        new_person = {
            'name': name,
            'embeddings': [embedding],
            'first_seen': datetime.now().isoformat(),
            'last_seen': datetime.now().isoformat(),
            'times_seen': 1
        }

        self.database['people'].append(new_person)
        self._save_database()
        print(f"✓ Added new person: {name}")
        return True

    def get_all_people(self) -> List[str]:
        """Get list of all known people"""
        return [person['name'] for person in self.database['people']]

    def get_person_info(self, name: str) -> Optional[Dict]:
        """Get information about a person"""
        for person in self.database['people']:
            if person['name'].lower() == name.lower():
                return {
                    'name': person['name'],
                    'first_seen': person.get('first_seen', 'Unknown'),
                    'last_seen': person.get('last_seen', 'Unknown'),
                    'times_seen': person.get('times_seen', 0),
                    'num_embeddings': len(person.get('embeddings', []))
                }
        return None

    def delete_person(self, name: str) -> bool:
        """Delete a person from database"""
        for i, person in enumerate(self.database['people']):
            if person['name'].lower() == name.lower():
                self.database['people'].pop(i)
                self._save_database()
                print(f"✓ Deleted {name} from database")
                return True
        print(f"⚠️  {name} not found in database")
        return False


def test_face_recognition():
    """Test the face recognition module"""
    print("\n" + "="*60)
    print("Face Recognition Module Test")
    print("="*60)

    # Initialize recognizer
    recognizer = FaceRecognizer()

    # Test with webcam if available
    print("\nTesting with webcam...")
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("⚠️  No webcam available")
        return

    print("Press 'q' to quit, 's' to save face, 'r' to recognize")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # Detect faces
        faces = recognizer.detect_faces(frame)

        # Draw rectangles around faces
        for (x, y, w, h) in faces:
            cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
            cv2.putText(frame, 'Face', (x, y-10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

        cv2.putText(frame, f'Faces: {len(faces)}', (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        cv2.imshow('Face Recognition Test', frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('s') and len(faces) > 0:
            # Save first detected face
            x, y, w, h = faces[0]
            face_img = frame[y:y+h, x:x+w]
            name = input("Enter name for this person: ")
            recognizer.add_person(name, face_img)
        elif key == ord('r') and len(faces) > 0:
            # Recognize first detected face
            x, y, w, h = faces[0]
            face_img = frame[y:y+h, x:x+w]
            name, confidence = recognizer.recognize_face(face_img)
            if name:
                print(f"Recognized: {name} (confidence: {confidence:.2f})")
            else:
                print(f"Unknown person (best match: {confidence:.2f})")

    cap.release()
    cv2.destroyAllWindows()

    # Print database stats
    print("\n" + "="*60)
    print("Database Statistics")
    print("="*60)
    people = recognizer.get_all_people()
    print(f"Total people: {len(people)}")
    for name in people:
        info = recognizer.get_person_info(name)
        print(f"\n{name}:")
        print(f"  First seen: {info['first_seen']}")
        print(f"  Last seen: {info['last_seen']}")
        print(f"  Times seen: {info['times_seen']}")
        print(f"  Embeddings: {info['num_embeddings']}")


if __name__ == '__main__':
    test_face_recognition()
