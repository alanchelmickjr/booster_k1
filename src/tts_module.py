#!/usr/bin/env python3
"""
Text-to-Speech Module for Booster K1
Simple TTS with fallback options
"""

import os
import threading
import queue

class TextToSpeech:
    """Simple TTS with multiple backend support"""

    def __init__(self, engine='espeak'):
        self.engine = engine
        self.available = False
        self.speech_queue = queue.Queue()
        self.worker_thread = None
        self.running = False

        # Try to initialize TTS engine
        if engine == 'espeak':
            self.available = self._test_espeak()
            if self.available:
                print("✓ Using espeak for TTS")
        elif engine == 'pyttsx3':
            self.available = self._test_pyttsx3()
            if self.available:
                print("✓ Using pyttsx3 for TTS")
        else:
            print(f"⚠️  Unknown TTS engine: {engine}")

        if not self.available:
            print("⚠️  TTS not available. Install espeak: sudo apt-get install espeak")

        # Start worker thread
        if self.available:
            self.start()

    def _test_espeak(self):
        """Test if espeak is available"""
        try:
            result = os.system('which espeak > /dev/null 2>&1')
            return result == 0
        except:
            return False

    def _test_pyttsx3(self):
        """Test if pyttsx3 is available"""
        try:
            import pyttsx3
            self.pyttsx3_engine = pyttsx3.init()
            return True
        except:
            return False

    def speak(self, text, blocking=False):
        """
        Speak the given text
        If blocking=True, wait for speech to complete
        """
        if not self.available:
            print(f"[TTS unavailable] {text}")
            return

        if blocking:
            self._do_speak(text)
        else:
            self.speech_queue.put(text)

    def _do_speak(self, text):
        """Actually perform the speech"""
        if self.engine == 'espeak':
            # Use espeak system command
            # Escape single quotes in text
            text = text.replace("'", "'\\''")
            os.system(f"espeak '{text}' 2>/dev/null")
        elif self.engine == 'pyttsx3':
            self.pyttsx3_engine.say(text)
            self.pyttsx3_engine.runAndWait()

    def _worker(self):
        """Worker thread that processes speech queue"""
        while self.running:
            try:
                text = self.speech_queue.get(timeout=0.5)
                self._do_speak(text)
            except queue.Empty:
                continue

    def start(self):
        """Start the TTS worker thread"""
        if self.running:
            return
        self.running = True
        self.worker_thread = threading.Thread(target=self._worker, daemon=True)
        self.worker_thread.start()

    def stop(self):
        """Stop the TTS worker thread"""
        self.running = False
        if self.worker_thread:
            self.worker_thread.join(timeout=1.0)

    def ask_question(self, question):
        """Ask a question and return (for future: could integrate with speech recognition)"""
        self.speak(question)
        # For now, just speak. Later we can add speech recognition to get response


def test_tts():
    """Test the TTS module"""
    print("\n" + "="*60)
    print("Text-to-Speech Module Test")
    print("="*60)

    # Test espeak
    print("\nTesting espeak...")
    tts = TextToSpeech(engine='espeak')

    if tts.available:
        print("Speaking test message...")
        tts.speak("Hello, I am the Booster K1 robot.", blocking=True)
        print("✓ TTS test complete")
    else:
        print("✗ TTS not available")

    tts.stop()


if __name__ == '__main__':
    test_tts()
