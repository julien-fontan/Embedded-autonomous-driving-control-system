import RPi.GPIO as GPIO

class MotorController:
    def __init__(self):
        # Pins pour le moteur unique
        self.EN = 18  # Pins for the single motor
        self.IN1 = 20
        self.IN2 = 16

        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(self.EN, GPIO.OUT)
        GPIO.setup(self.IN1, GPIO.OUT)
        GPIO.setup(self.IN2, GPIO.OUT)

        # PWM pour la vitesse (100% par défaut)
        self.pwm = GPIO.PWM(self.EN, 100)  # PWM for speed (100% by default)
        self.pwm.start(100)

        self.stop()

    def left(self):
        # Tourner à gauche (sens 1)
        GPIO.output(self.IN1, GPIO.HIGH)  # Turn left (direction 1)
        GPIO.output(self.IN2, GPIO.LOW)

    def right(self):
        # Tourner à droite (sens 2)
        GPIO.output(self.IN1, GPIO.LOW)  # Turn right (direction 2)
        GPIO.output(self.IN2, GPIO.HIGH)

    def stop(self):
        # Arrêt du moteur
        GPIO.output(self.IN1, GPIO.LOW)  # Stop the motor
        GPIO.output(self.IN2, GPIO.LOW)

    def set_steering(self, offset, max_offset=200):
        """
        Proportional control of the motor according to the offset.
        offset: deviation (positive = right, negative = left)
        max_offset: maximum expected value for the offset (for normalization)
        """
        # Clamp offset
        offset = max(-max_offset, min(max_offset, offset))
        # Augmentation de la puissance minimale à 50% (au lieu de 30%)
        # et utilisation de 100% plus rapidement
        duty_cycle = min(60, 50 + int(50 * abs(offset) / max_offset))  # 50% min, 100% max

        if offset < -50:
            # Gauche proportionnelle - CORRECTION : on tourne à droite quand on est à gauche de la route
            GPIO.output(self.IN1, GPIO.LOW)  # Proportional left - CORRECTION: turn right when on the left of the road
            GPIO.output(self.IN2, GPIO.HIGH)
            self.pwm.ChangeDutyCycle(duty_cycle)
        elif offset > 50:
            # Droite proportionnelle - CORRECTION : on tourne à gauche quand on est à droite de la route
            GPIO.output(self.IN1, GPIO.HIGH)  # Proportional right - CORRECTION: turn left when on the right of the road
            GPIO.output(self.IN2, GPIO.LOW)
            self.pwm.ChangeDutyCycle(duty_cycle)
        else:
            # Centré
            self.stop()  # Centered

    def __del__(self):
        self.stop()
        GPIO.cleanup()