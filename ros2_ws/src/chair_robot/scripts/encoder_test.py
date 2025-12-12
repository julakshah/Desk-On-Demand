#!/usr/bin/env python3

import sys
import math
#import pigpio
import lgpio

class EncoderRead:
    def __init__(self):
        self.gpio_pin_l = 4
        self.gpio_pin_r = 5

        self.wheel_l_OD = 0.05 ### CHANGE TO REAL VAL
        self.wheel_r_OD = 0.05 ### CHANGE TO REAL VAL
        self.track_width = 0.1 ### CHANGE TO REAL VAL

        # Configure pins for input and no pull up
        #self.pi = pigpio.pi()
        #self.pi.set_mode(self.gpio_pin_l, pigpio.INPUT)
        #self.pi.set_mode(self.gpio_pin_r, pigpio.INPUT)
        #self.pi.set_pull_up_down(self.gpio_pin_l, pigpio.PUD_OFF)
        #self.pi.set_pull_up_down(self.gpio_pin_r, pigpio.PUD_OFF)

        self.chip = lgpio.gpiochip_open(4)
        if self.chip < 0:
            raise RuntimeError(f"Failed to open gpiochip0: {lgpio.error_text(self.chip)}")

        # claim pins for alerts on both edges
        err = lgpio.gpio_claim_alert(self.chip, self.gpio_pin_l, lgpio.BOTH_EDGES)
        if err < 0:
            raise RuntimeError(f"gpio_claim_alert left failed: {lgpio.error_text(err)}")

        err = lgpio.gpio_claim_alert(self.chip, self.gpio_pin_r, lgpio.BOTH_EDGES)
        if err < 0:
            raise RuntimeError(f"gpio_claim_alert right failed: {lgpio.error_text(err)}")

        self.get_angle_tick_l = None
        self.get_angle_tick_r = None

        self.past_rise_l = None
        self.past_rise_r = None
        self.rise_l = None
        self.rise_r = None
        self.fall_l = None
        self.fall_r = None
        
        self.angle_l = 0.0
        self.angle_r = 0.0
        self.ang_vel_l = 0.0
        self.ang_vel_r = 0.0

        # declare callback function
        #self.cb_l = self.pi.callback(self.gpio_pin_l, pigpio.EITHER_EDGE, self.callback_left)
        #self.cb_r = self.pi.callback(self.gpio_pin_r, pigpio.EITHER_EDGE, self.callback_right)

        self.cb_l = lgpio.callback(self.chip,
                                   self.gpio_pin_l,
                                   lgpio.BOTH_EDGES,
                                   self.callback_left)

        self.cb_r = lgpio.callback(self.chip,
                                   self.gpio_pin_r,
                                   lgpio.BOTH_EDGES,
                                   self.callback_right)

    def callback_left(self, chip, gpio, level_new, tick):
        print(f"cb left")
        if level_new == 1:
            self.past_rise_l = self.rise_l
            self.rise_l = tick
        elif level_new == 0:
            self.fall_l = tick

    def callback_right(self, chip, gpio, level_new, tick):
        print(f"cb right")
        if level_new == 1:
            self.past_rise_r = self.rise_r
            self.rise_r = tick
        elif level_new == 0:
            self.fall_r = tick

    def update_rpm(self, right=False):
        # Times are in microseconds
        if right:
            t0_rise = self.past_rise_r
            t_fall = self.fall_r
            t1_rise = self.rise_r

            if t0_rise is None or t1_rise is None or t_fall is None:
                return

            if self.get_angle_tick_r is None:
                self.get_angle_tick_r = t1_rise
                return

            past_angle_update_tick = self.get_angle_tick_r
            self.get_angle_tick_r = t1_rise
            angle_period = self.get_angle_tick_r - past_angle_update_tick
            #angle_period = pigpio.tickDiff(past_angle_update_tick,self.get_angle_tick_r)
        else:
            t0_rise = self.past_rise_l
            t_fall = self.fall_l
            t1_rise = self.rise_l

            if t0_rise is None or t1_rise is None or t_fall is None:
                return

            if self.get_angle_tick_l is None:
                self.get_angle_tick_l = t1_rise
                return
            
            past_angle_update_tick = self.get_angle_tick_l
            self.get_angle_tick_l = t1_rise
            angle_period = self.get_angle_tick_l - past_angle_update_tick
            #angle_period = pigpio.tickDiff(past_angle_update_tick,self.get_angle_tick_l)
        
        if angle_period == 0:
            return
        # Convert to RPM given sensor behavior function
        # Using numbers from https://github.com/RobTillaart/AS5600 
        
        period = t1_rise - t0_rise
        #period = pigpio.tickDiff(t0_rise,t1_rise)
        dutyCycle = (1.0 * (t_fall - t0_rise)) / period
        angle = (dutyCycle - 0.0294) * (359.9 / (0.9706 - 0.0294))

        if right:
            past_angle = self.angle_r
            self.angle_r = angle # degrees
            # rad / s
            delta_angle_deg = (angle - past_angle + 540.0) % 360.0 - 180.0
            delta_angle_rad = math.radians(delta_angle_deg)
            self.ang_vel_r = delta_angle_rad * 1e6 / angle_period
            #self.ang_vel_r = (math.pi * 1e6 * ((angle - past_angle) % 360) / 180.0) / (angle_period)
        else:
            past_angle = self.angle_l
            self.angle_l = angle # degrees
            # rad / s
            delta_angle_deg = (angle - past_angle + 540.0) % 360.0 - 180.0
            delta_angle_rad = math.radians(delta_angle_deg)
            self.ang_vel_l = delta_angle_rad * 1e6 / angle_period
            #self.ang_vel_l = (math.pi * 1e6 * ((angle - past_angle) % 360) / 180.0) / (angle_period)

    @property
    def v_l(self):
        """ 
        (float): velocity of the left wheel, in m/s
        """
        return self.ang_vel_l * 0.5 * self.wheel_l_OD
    
    @property
    def v_r(self):
        """ 
        (float): velocity of the right wheel, in m/s
        """
        return self.ang_vel_r * 0.5 * self.wheel_r_OD
    
    @property
    def vel(self):
        """ 
        (float): linear velocity of the robot, in m/s
        """
        return 0.5 * (self.v_l + self.v_r)

    @property
    def angular_vel(self):
        """ 
        (float): angular velocity of the robot, in rad/s
            Counterclockwise is positive
        """
        return (self.v_r - self.v_l) / self.track_width

if __name__ == "__main__":
    enc = EncoderRead()
    try:
        count = 0
        while True:
            count += 1
            enc.update_rpm(right=False)
            enc.update_rpm(right=True)
            if (enc.v_l != 0 or enc.v_r != 0) and count % 100 == 0:
                print(f"Velocities: left {enc.v_l}, right {enc.v_r}, linear {enc.vel}, angular {enc.angular_vel}")
    except KeyboardInterrupt:
        print("Stopping")