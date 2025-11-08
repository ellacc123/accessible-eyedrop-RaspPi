# play short tunes or sound alerts through buzzer (success/fail tune) 

import RPi.GPIO as GPIO
import time

# GPIO pin connected to the buzzer
buzzer_pin = 23

# Use BCM numbering
GPIO.setmode(GPIO.BCM)
GPIO.setup(buzzer_pin, GPIO.OUT)

# Initialize PWM (initial frequency can be set arbitrarity)
pwm = GPIO.PWM(buzzer_pin, 440)
pwm.start(0)  # start silent 

# Define frequencies corresponding to musical notes
notes = {
    'do': 261,       # C4
    're': 293,       # D4
    'mi': 329,       # E4
    'fa': 349,       # F4
    'so': 392,       # G4
    'la': 440,       # A4
    'si': 493,       # B4
    'do_high': 523   # C5
}

def play_melody(state, note_duration=0.05, pause=0.05):
    """
    Play a melody based on the state:
      - state="success" plays ['do', 're', 'mi', 'so']
      - state="fail" plays ['so', 'mi', 're', 'do']
    """
    if state == "success":
        melody = ['do', 're', 'mi', 'so']
    elif state == "fail":
        melody = ['so', 'mi', 're', 'do']
    else:
        print("Unknown state")
        return
    
    for note in melody:
        if note in notes:
            freq = notes[note]
            pwm.ChangeFrequency(freq)
            pwm.ChangeDutyCycle(50)  # sound ON
        else:
            pwm.ChangeDutyCycle(0)   # Silence if note is undefined
        time.sleep(note_duration)
        # Brief silence after each note
        pwm.ChangeDutyCycle(0)
        time.sleep(pause)
    # pwm.stop()
    # GPIO.cleanup()

# Example usage:

#     # play success melody
# print("playing success sound")
# play_melody("success")

# time.sleep(1)  # wait 1 second

# # Play failure melody 
# print("Playing failure sound")
# play_melody("fail")

