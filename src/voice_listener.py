#!/usr/bin/env python3
"""
Simple Voice Listener for Booster K1
Uses SpeechRecognition to listen for spoken responses
"""

import speech_recognition as sr
import threading
import queue
import time

class VoiceListener:
    """Simple voice listener using SpeechRecognition"""

    def __init__(self, callback=None):
        """
        Initialize voice listener
        callback: function to call with recognized text
        """
        self.callback = callback
        self.recognizer = sr.Recognizer()
        self.microphone = None
        self.available = False
        self.running = False
        self.listener_thread = None

        # Queue for processing results
        self.result_queue = queue.Queue()

        # Test microphone availability
        try:
            self.microphone = sr.Microphone()
            self.available = True
            print("✓ Microphone available for voice input")
        except Exception as e:
            print(f"⚠️  Microphone not available: {e}")
            self.available = False

    def listen_once(self, timeout=5, phrase_time_limit=5):
        """
        Listen for a single phrase
        Returns recognized text or None
        """
        if not self.available:
            print("⚠️  Voice listener not available")
            return None

        try:
            with self.microphone as source:
                print("🎤 Listening...")
                # Adjust for ambient noise
                self.recognizer.adjust_for_ambient_noise(source, duration=0.5)

                # Listen for audio
                audio = self.recognizer.listen(
                    source,
                    timeout=timeout,
                    phrase_time_limit=phrase_time_limit
                )

                print("🔄 Processing speech...")

                # Recognize speech using Google Speech Recognition
                text = self.recognizer.recognize_google(audio)
                print(f"✓ Heard: {text}")
                return text

        except sr.WaitTimeoutError:
            print("⏱️  Listening timeout - no speech detected")
            return None
        except sr.UnknownValueError:
            print("❌ Could not understand audio")
            return None
        except sr.RequestError as e:
            print(f"❌ Speech recognition error: {e}")
            return None
        except Exception as e:
            print(f"❌ Error listening: {e}")
            return None

    def ask_and_listen(self, tts, question, timeout=5, phrase_time_limit=5):
        """
        Ask a question via TTS and listen for response
        Returns recognized text or None
        """
        if not self.available:
            return None

        # Ask the question
        if tts and tts.available:
            tts.speak(question, blocking=True)
            # Small pause after speaking
            time.sleep(0.3)

        # Listen for response
        return self.listen_once(timeout=timeout, phrase_time_limit=phrase_time_limit)

    def start_continuous_listening(self):
        """Start continuous listening in background thread"""
        if not self.available or self.running:
            return

        self.running = True
        self.listener_thread = threading.Thread(target=self._continuous_listen, daemon=True)
        self.listener_thread.start()
        print("🎤 Continuous listening started")

    def stop_continuous_listening(self):
        """Stop continuous listening"""
        self.running = False
        if self.listener_thread:
            self.listener_thread.join(timeout=2.0)
        print("🔇 Continuous listening stopped")

    def _continuous_listen(self):
        """Continuous listening loop (runs in background thread)"""
        with self.microphone as source:
            # Adjust for ambient noise once
            self.recognizer.adjust_for_ambient_noise(source, duration=1.0)

            while self.running:
                try:
                    # Listen for audio
                    audio = self.recognizer.listen(source, timeout=1, phrase_time_limit=5)

                    # Process in background
                    threading.Thread(
                        target=self._process_audio,
                        args=(audio,),
                        daemon=True
                    ).start()

                except sr.WaitTimeoutError:
                    continue
                except Exception as e:
                    if self.running:
                        print(f"⚠️  Listening error: {e}")
                    time.sleep(0.5)

    def _process_audio(self, audio):
        """Process audio in background"""
        try:
            text = self.recognizer.recognize_google(audio)
            print(f"🎤 Heard: {text}")

            # Call callback if provided
            if self.callback:
                self.callback(text)

        except sr.UnknownValueError:
            pass  # Ignore unintelligible audio
        except Exception as e:
            print(f"⚠️  Recognition error: {e}")


def test_voice_listener():
    """Test the voice listener"""
    print("\n" + "="*60)
    print("Voice Listener Test")
    print("="*60)

    # Test callback function
    def on_speech(text):
        print(f"[Callback] Recognized: {text}")

    # Create listener
    listener = VoiceListener(callback=on_speech)

    if not listener.available:
        print("❌ Voice listener not available")
        return

    # Test 1: Listen once
    print("\nTest 1: Say something...")
    text = listener.listen_once(timeout=5)
    if text:
        print(f"✓ Successfully recognized: {text}")
    else:
        print("✗ No speech recognized")

    # Test 2: Ask and listen
    print("\nTest 2: Ask and listen...")
    from tts_module import TextToSpeech
    tts = TextToSpeech(engine='espeak')

    response = listener.ask_and_listen(
        tts,
        "What is your name?",
        timeout=5,
        phrase_time_limit=5
    )

    if response:
        print(f"✓ You said: {response}")
        tts.speak(f"Nice to meet you, {response}!", blocking=True)
    else:
        print("✗ No response heard")

    tts.stop()
    print("\n✓ Test complete")


if __name__ == '__main__':
    test_voice_listener()
