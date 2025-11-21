from gpiozero import PWMLED, LED
from time import sleep

#motor = PWMLED(18)   # ENA
#high  = LED(17)      # IN1
#low   = LED(22)      # IN2

motor = PWMLED(27)   # ENA
high  = LED(23)      # IN1
low   = LED(24)  

print("Spinning motor")
high.on()      # IN1 = 1
low.off()      # IN2 = 0
motor.value = 0.5
sleep(1)

for i in range(100):
    high.on()      # IN1 = 1
    low.off()      # IN2 = 0
    motor.value = 0.5 * (i / 100)
    sleep(0.05)

sleep(1.0)
print("Swapping direction")
for i in range(100):
    high.off()      # IN1 = 1
    low.on()      # IN2 = 0
    motor.value = 0.5 * ((100-i) / 100)
    sleep(0.05)

print("Stop")
motor.value = 0
high.off()
low.off()
