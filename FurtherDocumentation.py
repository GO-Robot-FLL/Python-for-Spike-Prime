"""
SPIKE Prime Python Classes
"""
class App:
    def __init__(self):
        pass
    def play_sound(self, name, volume=100):
        """
        Plays a sound from the device (tablet or computer).
        
        The program will not continue until the sound has finished playing.
        
        If no sound is found with the given name is found, nothing will happen.
        
        Parameters
        -------------
        name : The name of the sound to play.
        
        Type : string (text)
        
        Values : Alert, Applause1, Applause2, Applause3, Baa, Bang1, Bang2, BasketballBounce, BigBoing, Bird, Bite, BoatHorn1, BoatHorn2, Bonk, BoomCloud, BoopBingBop, BowlingStrike, Burp1, Burp2, Burp3, CarAccelerate1, CarAccelerating2, CarHorn, CarIdle, CarReverse, CarSkid1, CarSkid2, CarVroom, CatAngry, CatHappy, CatHiss, CatMeow1, CatMeow2, CatMeow3, CatPurring, CatWhining, Chatter, Chirp, Clang, ClockTicking, ClownHonk1, ClownHonk2, ClownHonk3, Coin, Collect, Connect, Crank, CrazyLaugh, Croak, CrowdGasp, Crunch, Cuckoo, CymbalCrash, Disconnect, DogBark1, DogBark2, DogBark3, DogWhining1, DogWhining2, Doorbell1, Doorbell2, Doorbell3, DoorClosing, DoorCreek1, DoorCreek2, DoorHandle, DoorKnock, DoorSlam1, DoorSlam2, DrumRoll, DunDunDunnn, EmotionalPiano, Fart1, Fart2, Fart3, Footsteps, Gallop, GlassBreaking, Glug, GoalCheer, Gong, Growl, Grunt, HammerHit, HeadShake, HighWhoosh, Jump, JungleFrogs, Laser1, Laser2, Laser3, LaughingBaby1, LaughingBaby2, LaughingBoy, LaughingCrowd1, LaughingCrowd2, LaughingGirl, LaughingMale, Lose, LowBoing, LowSqueak, LowWhoosh, MagicSpell, MaleJump1, MaleJump2, Moo, OceanWave, Oops, OrchestraTuning, PartyBlower, Pew, PingPongHit, PlaneFlyingBy, PlaneMotorRunning, PlaneStarting, Pluck, PoliceSiren1, PoliceSiren2, PoliceSiren3, Punch, Rain, Ricochet, Rimshot, RingTone, Rip, Robot1, Robot2, Robot3, RocketExplosionRumble, Rooster, ScramblingFeet, Screech, Seagulls, ServiceAnnouncement, SewingMachine, ShipBell, SirenWhistle, Skid, SlideWhistle1, SlideWhistle2, SneakerSqueak, Snoring, Snort, SpaceAmbience, SpaceFlyby, SpaceNoise, Splash, SportWhistle1, SportWhistle2, SqueakyToy, SquishPop, SuctionCup, Tada, TelephoneRing2, TelephoneRing, Teleport2, Teleport3, Teleport, TennisHit, ThunderStorm, TolietFlush, ToyHonk, ToyZing, Traffic, TrainBreaks, TrainHorn1, TrainHorn2, TrainHorn3, TrainOnTracks, TrainSignal1, TrainSignal2, TrainStart, TrainWhistle, Triumph, TropicalBirds, Wand, WaterDrop, WhistleThump, Whiz1, Whiz2, WindowBreaks, Win, Wobble, WoodTap, Zip
        
        Default : no default value
        
        volume : The volume at which the sound will be played.
        
        Type : integer (positive or negative whole number, including 0)
        
        Values : 0 to 100 %
        
        Default : 100%
        
        Errors
        ------------
        TypeError : name is not a string or volume is not an integer.
        
        RuntimeError : The SPIKE App has been disconnected from the Hub.
        """
        pass
    def start_sound(self, name, volume):
        """
        Starts playing a sound from your device (tablet or computer).
        The program will not wait for the sound to finish playing before proceeding to the next command.
        If no sound with the given name is found, nothing will happen.
        
        Parameters
        -------------
        name : The name of the sound to play.
        
        Type : string (text)
        
        Values : Alert, Applause1, Applause2, Applause3, Baa, Bang1, Bang2, BasketballBounce, BigBoing, Bird, Bite, BoatHorn1, BoatHorn2, Bonk, BoomCloud, BoopBingBop, BowlingStrike, Burp1, Burp2, Burp3, CarAccelerate1, CarAccelerating2, CarHorn, CarIdle, CarReverse, CarSkid1, CarSkid2, CarVroom, CatAngry, CatHappy, CatHiss, CatMeow1, CatMeow2, CatMeow3, CatPurring, CatWhining, Chatter, Chirp, Clang, ClockTicking, ClownHonk1, ClownHonk2, ClownHonk3, Coin, Collect, Connect, Crank, CrazyLaugh, Croak, CrowdGasp, Crunch, Cuckoo, CymbalCrash, Disconnect, DogBark1, DogBark2, DogBark3, DogWhining1, DogWhining2, Doorbell1, Doorbell2, Doorbell3, DoorClosing, DoorCreek1, DoorCreek2, DoorHandle, DoorKnock, DoorSlam1, DoorSlam2, DrumRoll, DunDunDunnn, EmotionalPiano, Fart1, Fart2, Fart3, Footsteps, Gallop, GlassBreaking, Glug, GoalCheer, Gong, Growl, Grunt, HammerHit, HeadShake, HighWhoosh, Jump, JungleFrogs, Laser1, Laser2, Laser3, LaughingBaby1, LaughingBaby2, LaughingBoy, LaughingCrowd1, LaughingCrowd2, LaughingGirl, LaughingMale, Lose, LowBoing, LowSqueak, LowWhoosh, MagicSpell, MaleJump1, MaleJump2, Moo, OceanWave, Oops, OrchestraTuning, PartyBlower, Pew, PingPongHit, PlaingFlyingBy, PlaneMotorRunning, PlaneStarting, Pluck, PoliceSiren1, PoliceSiren2, PoliceSiren3, Punch, Rain, Ricochet, Rimshot, RingTone, Rip, Robot1, Robot2, Robot3, RocketExplosionRumble, Rooster, ScramblingFeet, Screech, Seagulls, ServiceAnnouncement, SewingMachine, ShipBell, SirenWhistle, Skid, SlideWhistle1, SlideWhistle2, SneakerSqueak, Snoring, Snort, SpaceAmbience, SpaceFlyby, SpaceNoise, Splash, SportWhistle1, SportWhistle2, SqueakyToy, SquishPop, SuctionCup, Tada, TelephoneRing2, TelephoneRing, Teleport2, Teleport3, Teleport, TennisHit, ThunderStorm, TolietFlush, ToyHonk, ToyZing, Traffic, TrainBreaks, TrainHorn1, TrainHorn2, TrainHorn3, TrainOnTracks, TrainSignal1, TrainSignal2, TrainStart, TrainWhistle, Triumph, TropicalBirds, Wand, WaterDrop, WhistleThump, Whiz1, Whiz2, WindowBreaks, Win, Wobble, WoodTap, Zip
        
        Default : no default value
        
        volume : The volume at which the sound will be played.
        
        Type : integer (positive or negative whole number, including 0)
        
        Values : 0 to 100 %
        
        Default : 100%
        
        Errors
        ------------
        TypeError : name is not a string or volume is not an integer.
        
        RuntimeError : The SPIKE App has been disconnected from the Hub.
        """
        pass

class DistanceSensor:
    def __init__(self, port):
        """
        Allows access to the connected distance sensor.
        Parameters
        -------------
        port : The port label the sensor is connected to
        
        Type : string (text)
        
        Values : 'A', 'B', 'C', 'D', and 'E'
        
        Default : no default value
        """
        pass
    def light_up_all(self, brightness=100):
        """
        Lights up all of the lights on the Distance Sensor with the specified brightness.
        Parameters
        ---------
        brightness : The specified brightness of all the lights.
        
        Type : integer (positive or negative whole number, including 0)
        
        Values : 0 to 100 % (0 is off and 100 is full brightness.)
        
        Default : 100 %
        
        Errors
        ---------- 
        TypeError : brightness is not an integer.
        
        RuntimeError : The sensor has been disconnected from the Port.
        """
        pass
    def light_up(self, right_top=100, left_top=100, right_bottom=100, left_bottom=100):
        """
        Sets the brightness of the individual lights on the Distance Sensor.
        
        Parameters
        -------------
        right_top : The brightness of the light above the right part of the Distance Sensor.
        
        Type : integer (positive or negative whole number, including 0)
        
        Values : 0 to 100% (0 is off and 100 is full brightness.)
        
        Default : 100
        
        left_top : The brightness of the light above the left part of the Distance Sensor.
        
        Type : integer (positive or negative whole number, including 0)
        
        Values : 0 to 100% (0 is off and 100 is full brightness.)
        
        Default : 100
        
        right_bottom : The brightness of the light below the right part of the Distance Sensor.
        
        Type : integer (positive or negative whole number, including 0)
        
        Values : 0 to 100% (0 is off and 100 is full brightness.)
        
        Default : 100
        
        left_bottom : The brightness of the light below the left part of the Distance Sensor.
        
        Type : integer (positive or negative whole number, including 0)
        
        Values : 0 to 100% (0 is off and 100 is full brightness.)
        
        Default : 100
        
        Errors
        ---------------
        TypeError : right_top, left_top, right_bottom or left_bottom is not a number.
        
        RuntimeError : The sensor has been disconnected from the Port.
        """
        pass
    def get_distance_cm(self, short_range=False):
        """
        Retrieves the measured distance in centimeters.
        
        Parameters
        ----------
        short_range : Whether to use or not the short range mode. The short range mode increases accuracy but it can only detect nearby objects.
        
        Type : boolean
        
        Values : True or False
        
        Default : False
        
        Returns
        ----------------
        The measured distance, or "none" if the distance cannot be measured.
        Type : float (decimal number)
        
        Values : 0 to 200 cm
        
        Errors
        ------------
        TypeError : short_range is not a boolean.
        RuntimeError :  The sensor has been disconnected from the Port.
        """
        pass
    def get_distance_inches(self, short_range=False):
        """
        Gets the measured distance in inches.
        
        Parameters
        ----------
        short_range : Whether or not to use short range mode. Short range mode increases accuracy but it can only detect nearby objects.
        
        Type : boolean
        
        Values : True or False
        
        Default : False
        
        Returns
        ------------------
        The measured distance, or "none" if the distance cannot be measured.
        
        Type : float (decimal number)
        
        Values : any value between 0 and 79
        
        Errors
        -------------
        TypeError : short_range is not a boolean.
        
        RuntimeError : The sensor has been disconnected from the Port.
        """
        pass

    def get_distance_percentage(self, short_range=False):
        """
        Retrieves the measured distance in percent.
        
        Parameters
        ---------
        short_range : Whether or not to use short range mode. Short range mode increases accuracy but it can only detect nearby objects.
        
        Type : boolean
        
        Values : True or False
        
        Default : False
        
        Returns
        ----------
        The measured distance, or "none" if the distance cannot be measured.
        
        Type : integer (positive or negative whole number, including 0)
        
        Values : any value between 0 and 100
        
        Errors
        ----------
        TypeError : short_range is not a boolean.
        
        RuntimeError : The sensor has been disconnected from the Port.
        """
        pass
    def wait_for_distance_farther_than(self, distance, unit='cm', short_range=False):
        """
        Waits until the measured distance is greater than distance.
        
        Parameters
        -----------
        distance : The target distance to detect, from the sensor to an object.
        
        Type : float (decimal number)
        
        Values : any value
        
        Default : no default value
        
        unit : Unit of measurement of the distance.
        
        Type : string (text)
        
        Values : 'cm','in','%'
        
        Default : cm
        
        short_range : Whether or not to use short range mode. Short range mode increases accuracy but it can only detect nearby objects.
        
        Type : boolean
        
        Values : True or False
        
        Default : False
        
        Errors
        ----------
        TypeError : distance is not a number or unit is not a string or short_range is not a boolean.
        
        ValueError : unit is not one of the allowed values.
        
        RuntimeError : The sensor has been disconnected from the Port.
        """
        pass
    def wait_for_distance_closer_than(self, distance, unit='cm', short_range=False):
        """
        Waits until the measured distance is less than distance.
        
        Parameters
        --------------
        distance : The target distance to detect, from the sensor to an object.
        
        Type : float (decimal number)
        
        Values : any value
        
        Default : no default value
        
        unit : Unit of measurement of the distance.
        
        Type : string (text)
        
        Values : 'cm','in','%'
        
        Default : cm
        
        short_range : Whether or not to use short range mode. Short range mode increases accuracy but it can only detect nearby objects.
        
        Type : boolean
        
        Values : True or False
        
        Default : False
        
        Errors
        ------------
        TypeError : distance is not a number or unit is not a string or short_range is not a boolean.
        
        ValueError : unit is not one of the allowed values. short_range is not a boolean.
        
        RuntimeError: The sensor has been disconnected from the Port.
        """
        pass

class ForceSensor:
    def __init__(self, port):
        """
        Allows access to the connected force sensor.
        Parameters
        -------------
        port : The port label the sensor is connected to
        
        Type : string (text)
        
        Values : 'A', 'B', 'C', 'D', and 'E'
        
        Default : no default value
        """
        pass
    def is_pressed(self):
        """
        Tests whether the button on the sensor is pressed.
        
        Returns
        ----------
        True if the button is pressed.
        
        Type : boolean
        
        Values : True or False
        
        Errors
        -----------
        RuntimeError : The Force Sensor has been disconnected from the port.
        """
        pass
    def get_force_newton(self):
        """
        Retrieves the measured force, in newtons.
        
        Returns
        -----------
        The measured force in newtons.
        
        Type : float (decimal number)
        
        Values : between 0 and 10
        
        Errors
        ------------
        RuntimeError : The Force Sensor has been disconnected from the port.
        """
        pass
    def get_force_percentage(self):
        """
        Retrieves the measured force as a percentage of the maximum force.
        
        Returns
        ------------
        The measured force, in percentage.
        
        Type : integer (positive or negative whole number, including 0)
        
        Values : between 0 - 100%.
        
        Errors
        -----------
        RuntimeError : The Force Sensor has been disconnected from the port.
        """
        pass
    def wait_until_pressed(self):
        """
        Waits until the Force Sensor is pressed.
        Errors
        ----------
        RuntimeError : The sensor has been disconnected from the port.
        """
        pass
    def wait_until_released(self):
        """
        Waits until the Force Sensor is released.
        Errors
        ---------
        RuntimeError : The sensor has been disconnected from the Port.
        """
        pass

class LightMatrix:
    def __init__(self):
        """
          Controls the light matrix on the Spike Hub
        """
        pass
    def show_image(self, image, brightness=100):
        """
        Shows an image on the Light Matrix.
        
        Parameters
        -----------
        image : Name of the image.
        
        Type : string (text)
        
        Values : ANGRY, ARROW_E, ARROW_N, ARROW_NE, ARROW_NW, ARROW_S, ARROW_SE, ARROW_SW, ARROW_W, ASLEEP, BUTTERFLY, CHESSBOARD, CLOCK1, CLOCK10, CLOCK11, CLOCK12, CLOCK2, CLOCK3, CLOCK4, CLOCK5, CLOCK6, CLOCK7, CLOCK8, CLOCK9, CONFUSED, COW, DIAMOND, DIAMOND_SMALL, DUCK, FABULOUS, GHOST, GIRAFFE, GO_RIGHT, GO_LEFT, GO_IP, GO_DOWN, HAPPY, HEART, HEART_SMALL, HOUSE, MEH, MUSIC_CROTCHET, MUSIC_QUAVER, MUSIC_QUAVERS, NO, PACMAN, PITCHFORK, RABBIT, ROLLERSKATE, SAD, SILLY, SKULL, SMILE, SNAKE, SQUARE, SQUARE_SMALL, STICKFIGURE, SURPRISED, SWORD, TARGET, TORTOISE, TRIANGLE, TRIANGLE_LEFT, TSHIRT, UMBRELLA, XMAS, YES
        
        Default : no default value
        
        brightness : Brightness of the image
        
        Type : integer (positive or negative whole number, including 0)
        
        Values : 0 to 100%
        
        Default : 100
        
        Errors
        -------------------
        TypeError : image is not a string or brightness is not an integer.
        
        ValueError : image is not one of the allowed values.
        """
        pass
    def set_pixel(self, x, y, brightness=100):
        """
        Sets the brightness of one pixel (one of the 25 LED) on the Light Matrix.
        
        Parameters
        -----------
        x : Pixel position, counting from the left.
        
        Type : integer (positive or negative whole number, including 0)
        
        Values : 1 to 5
        
        Default : no default value
        
        y : Pixel position, counting from the top.
        
        Type : integer (positive or negative whole number, including 0)
        
        Values : 1 to 5
        
        Default : no default value
        
        brightness : Brightness of the pixel
        
        Type : integer (positive or negative whole number, including 0)
        
        Values : 0 to 100%
        
        Default : 100
        
        Errors
        --------------
        TypeError : x, y or brightness is not an integer.
        
        ValueError : x, y is not within the allowed range of 0-4.
        """
        pass
    def write(self, text):
        """
        Writes text on the Light Matrix, one letter at a time, scrolling from right to left.
        Your program will not continue until all of the letters have been shown.
    
        Parameters
        ---------
        text : Text to write.
    
        Type : string (text)
    
        Values : any text
    
        Default : no default value
        """
        pass
    def off(self):
        """
        Turns off all the pixels on the Light Matrix.
        """ 
        pass


class MotionSensor:
    def __init__(self):
        """
        Senses angle and motion changes and it built into the Hub.
        """
        pass
    def reset_yaw_angle(self):
        """
        Sets the yaw angle to 0.
        """
        pass
    def get_yaw_angle(self):
        """
        Retrieves the yaw angle of the Hub.
        
        "Yaw" is the rotation around the front-back (vertical) axis. "Pitch" the is rotation around the left-right (transverse) axis. "Roll" the is rotation around the front-back (longitudinal) axis.
        
        Returns
        -------
        The yaw angle, specified in degrees.
        Type : Integer (Positive or negative whole number, including 0)
        
        Values : -180 to 180 BUG: in 1.2 the values are -179 to 179
        """
        pass
    def get_orientation(self):
        """
        Retrieves the current orientation of the Hub.
        Returns
        -------
        The Hub's current orientation.
        Type : string (text)
        Values : 'front','back','up','down','leftside','rightside'
        """
        pass
    def get_gesture(self):
        """
        Retrieves the most recently-detected gesture.
        Returns
        ---------
        The gesture.
        Type : string (text)
        Values : 'shaken','tapped','doubletapped','falling'
        """
        pass
    def get_roll_angle(self):
        """
        Retrieves the roll angle of the Hub.
        "Roll" is the rotation around the front-back (longitudinal) axis. "Yaw" is the rotation around the front-back (vertical) axis. "Pitch" is the rotation around the left-right (transverse) axis.
        Returns
        -----------
        The roll angle, specified in degrees.
        Type : Integer (Positive or negative whole number, including 0)
    
        Values : -180 to 180 BUG: currently returns -179 to 179
        """
        pass
    def get_pitch_angle(self):
        """
        Retrieves the pitch angle of the Hub.
        "Pitch" is the rotation around the left-right (transverse) axis. "Roll" is the rotation around the front-back (longitudinal) axis. "Yaw" is the rotation around the front-back (vertical) axis.
        Returns
        -----------
        The pitch angle, specified in degrees.
        Type : Integer (Positive or negative whole number, including 0)
        Values : -180 to 180 BUG: only reads -90 to 90
        """
        pass
    def was_gesture(self, gesture):
        """
        Tests whether a gesture has occurred since the last time was_gesture() was used or since the beginning of the program (for the first use).
        Parameters
        ------------
        gesture : The name of the gesture.
        
        Type : string (text)
        
        Values : 'shaken','tapped','doubletapped','falling','None'
        
        Default : no default value
        
        Errors
        ------------
        TypeError : gesture is not a string.
        
        ValueError : gesture is not one of the allowed values.
        
        Returns
        ----------
        True if gesture has occurred since the last time was_gesture() was called, otherwise False.
        
        Type : boolean 
        
        Values : True or False
        """
        pass
    def wait_for_new_gesture(self):
        """
        Waits until a new gesture happens.
        Returns
        --------------
        The new gesture.
        Type : string (text)
        Values : 'shaken','tapped','doubletapped','falling'
        """
        pass
    def wait_for_new_orientation(self):
        """
        Waits until the orientation of the Hub changes.
        The first time this method is called, it will immediately return the current value. After that, calling this method will block the program until the Hub's orientation has changed since the previous time this method was called.
        Returns
        ----------
        The Hub's new orientation.
        Type : string (text)
        Values : 'front', 'back', 'up', 'down', 'left side', 'right side'
        """
        pass


class Motor:
    def __init__(self, port):
        """
        Controls a single motor
        Parameters
        -------------
        port : The port label the motor is connected to.
        
        Type : string (text)
        
        Values : 'A', 'B', 'C', 'D', and 'E'
        
        Default : no default value       
        """
        pass
    def run_to_position(self, degrees, direction='shortest path', speed=None):
        """
        Runs the motor to an absolute position.
        The sign of the speed will be ignored (absolute value) and the motor will always travel in the direction specified by direction parameter. If speed is greater than 100, it will be limited to 100.
        
        Parameters
        ------------
        degrees : The target position.
        
        Type : integer (positive or negative whole number, including 0)
        
        Values : 0 to 359
        
        Default : no default value
        
        direction : The direction to use to reach the target position.
        
        Type : string (text)
        
        Values : shortest path' could run in either direction depending on the shortest distance to the target. - 'clockwise' will make the motor run clockwise until it reaches the target position. - 'counterclockwise' will make the motor run counterclockwise until it reaches the target position.
        
        Default : no default value
        
        speed : The motor's speed.
        
        Type : integer (positive or negative whole number, including 0)
        
        Values : 0 to 100%
        
        Default : if no value is specified, it will use the default speed set by set_default_speed().
        
        Errors
        -----------------
        TypeError : degrees or speed is not an integer or direction is not a string.
        
        ValueError : direction is not one of the allowed values or degrees is not within the range of 0-359 (both inclusive).
        
        RuntimeError : The motor has been disconnected from the Port.
        """
        pass
    def run_to_degrees_counted(self, degrees, speed=None):
        """
        Runs the motor to until degrees counted is equal to the value specified by the degrees parameter.
        The sign of the speed will be ignored and the motor will always travel in the direction required to reach degrees. If speed is greater than 100, it will be limited to 100.
        
        Parameters
        -----------
        degrees : The target degrees counted.
        
        Type : integer (positive or negative whole number, including 0)
        
        Values : any number
        
        Default : no default value
        
        speed : The desired speed..
        
        Type : integer (positive or negative whole number, including 0)
        
        Values : 0 to 100% (positive values only)
        
        Default : if no value is specified, it will use the default speed set by set_default_speed().
        
        Errors
        -----------
        TypeError : degrees or speed is not an integer.
        
        RuntimeError : The motor has been disconnected from the Port.
        """
        pass
    def run_for_degrees(self, degrees, speed=None):
        """
        Runs the motor for a given number of degrees.
        
        Parameters
        -------------
        degrees : The number of degrees the motor should run.
        
        Type : integer (positive or negative whole number, including 0)
        
        Values : any number
        
        Default : no default value
        
        speed : The motor's speed
        
        Type : integer (positive or negative whole number, including 0)
        
        Values : -100% to 100%
        
        Default : if no value is specified, it will use the default speed set by set_default_speed().
        
        Errors
        ----------
        TypeError : degrees or speed is not an integer.
        
        RuntimeError : The motor has been disconnected from the Port.
        """
        pass
    def run_for_rotations(self, rotations, speed=None):
        """
        Runs the motor for a specified number of rotations.
        
        Parameters
        -------------
        rotations : The number of rotations the motor should run.
        
        Type : float (decimal number)
        
        Values : any number
        
        Default : no default value
        
        speed : The motor's speed
        
        Type : integer (positive or negative whole number, including 0)
        
        Values : -100% to 100%
        
        Default : if no value is specified, it will use the default speed set by set_default_speed().
        
        Errors
        -------------
        TypeError : rotations is not a number or speed is not an integer.
        
        RuntimeError : The motor has been disconnected from the Port.
        """
        pass
    def run_for_seconds(self, seconds, speed=None):
        """
        Runs the motor for a specified number of seconds.
    
        Parameters
        -----------
        seconds : The number of seconds for which the motor should run.
    
        Type : float (decimal number)
    
        Values : any number
    
        Default : no default value
    
        speed : The motor's speed.
    
        Type : integer (positive or negative whole number, including 0)
    
        Values : -100% to 100%
    
        Default : if no value is specified, it will use the default speed set by set_default_speed().
    
        Errors
        ----------
        TypeError : seconds is not a number or speed is not an integer.
    
        RuntimeError : The motor has been disconnected from the Port.
        """
        pass
    def start(self, speed=None):
        """
        Starts running the motor at a specified speed.
        The motor will keep moving at this speed until you give it another motor command, or when your program ends.
        
        Parameters
        ----------
        speed : The motor's speed.
        
        Type : integer (positive or negative whole number, including 0)
        
        Values : -100% to 100%
        
        Default : if no value is specified, it will use the default speed set by set_default_speed().
        
        Errors
        --------
        TypeError : speed is not an integer.
        
        RuntimeError : The motor has been disconnected from the Port.
        """
        pass
    def stop(self):
        """
        Stops the motor. What the motor does after it stops depends on the action set in set_stop_action(). The default value of set_stop_action() is coast.
        
        Errors
        ----------
        RuntimeError : The motor has been disconnected from the Port.
        """
        pass
    def start_at_power(self, power):
        """
        Starts rotating the motor at a specified power level.
        The motor will keep moving at this power level until you give it another motor command, or when your program ends.
        
        Parameters
        ---------
        power : Power of the motor.
        
        Type : integer (positive or negative whole number, including 0)
        
        Values : -100% to 100%
        
        Default : no default value
        
        Errors
        ---------
        TypeError : power is not an integer.
        
        RuntimeError : The motor has been disconnected from the Port.
        """
        pass
    def get_speed(self):
        """
        Retrieves the speed of the motor.
        
        Returns
        ----------
        The current speed of the motor
        
        Type : integer (positive or negative whole number, including 0)
        
        Values : -100% to 100%
        
        Errors
        ------------
        RuntimeError : The motor has been disconnected from the Port.
        """
        pass
    def get_position(self):
        """
        Retrieves the position of the motor. This is the clockwise angle between the moving marker and the zero-point marker on the motor.
        
        Returns
        ----------
        The position of the motor
        
        Type : integer (positive or negative whole number, including 0)
        
        Values : 0 to 359 degrees
        
        Errors
        ------------
        RuntimeError : The motor has been disconnected from the Port.
        """
        pass
    def get_degrees_counted(self):
        """
        Retrieves the degrees counted by the motor.
        
        Returns
        -----------
        The number of degrees counted.
        
        Type : integer (positive or negative whole number, including 0)
        
        Values : any number
        
        Errors
        ----------
        RuntimeError : The motor has been disconnected from the Port.
        """
        pass
    def get_default_speed(self):
        """
        Retrieves the current default motor speed.
        Returns
        ---------
        The default motor's speed.
        Type : integer (positive or negative whole number, including 0)
        Values : -100% to 100%.
        
        Errors
        ----------
        RuntimeError : The motor has been disconnected from the Port.
        """
        pass
    def was_interrupted(self):
        """
        Tests whether the motor was interrupted.
        
        Returns
        -----------
        True if the motor was interrupted since the last time was_interrupted() was called, otherwise False.
        
        Type : boolean
        
        Values : True or False
        
        Errors
        -----------
        RuntimeError : The motor has been disconnected from the Port.
        """
        pass
    def was_stalled(self):
        """
        Tests whether the motor was stalled.
        
        Returns
        -----------
        True if the motor was stalled since the last time was_stalled() was called, otherwise False.
        
        Type : boolean
        
        Values : True or False
        
        Errors
        -----------
        RuntimeError : The motor has been disconnected from the Port.
        """
        pass
    def set_degrees_counted(self, degrees_counted):
        """
        Sets the number of degrees counted to a desired value.
        
        Parameters
        ----------
        degrees_counted : The value to which the number of degrees counted should be set.
        
        Type :  integer (positive or negative whole number, including 0)
        
        Values : any number
        
        Default : no default value
        
        Errors
        ----------
        TypeError : degrees_counted is not an integer.
        
        RuntimeError : The motor has been disconnected from the Port.
        """
        pass
    def set_default_speed(self, default_speed):
        """
        Sets the default motor speed. This speed will be used when you omit the speed argument in one of the other methods, such as run_for_degrees.
        Setting the default speed does not affect any motors that are currently running.
        It will only have an effect when another motor method is called after this method.
        If the value of default_speed is outside of the allowed range, the default speed will be set to -100 or 100 depending on whether the value was negative or positive.
        
        Parameters
        -----------
        default_speed : The default speed value.
        
        Type : integer (positive or negative whole number, including 0)
        
        Values : -100% to 100%.
        
        Default : no default value
        
        Errors
        ------------
        TypeError : default_speed is not an integer.
        """
        pass

    def set_stop_action(self, action):
        """
        Sets the default behavior when a motor stops.
        
        Parameters
        -------------
        action : The desired motor behavior when the motor stops.
        
        Type : string (text)
        
        Values : 'coast','brake','hold'
        
        Default : coast
        
        Errors
        ------------
        TypeError : action is not a string.
        
        ValueError : action is not one of the allowed values.
        
        RuntimeError : The motor has been disconnected from the Port.
        """
        pass
    def set_stall_detection(self, stop_when_stalled):
        """
        Turns stall detection on or off.
        Stall detection senses when a motor has been blocked and can't move. If stall detection has been enabled and a motor is blocked, the motor will be powered off after two seconds and the current motor command will be interrupted. If stall detection has been disabled, the motor will keep trying to run and programs will "get stuck" until the motor is no longer blocked.
        Stall detection is enabled by default.
        
        Parameters
        -----------
        stop_when_stalled : Choose True to enable stall detection or False to disable it.
        
        Type : boolean
        
        Values : True or False
        
        Default : True
        
        Errors
        -----------
        TypeError : stop_when_stalled is not a boolean.
        
        RuntimeError : The motor has been disconnected from the Port.
        """
        pass

class MotorPair:
    def __init__(self, port1, port2):
        """
        Binds two motors together to use as a pair.
        Parameters
        ----------
        port1 : Port of the first motor
        Type : string (text)
        
        Values : 'A', 'B', 'C', 'D', and 'E'
        
        Default : no default value       
        port2 :  Port of the second motor
        Type : string (text)
        
        Values : 'A', 'B', 'C', 'D', and 'E'
        
        Default : no default value       
        """
        pass

    def move(self, amount, unit='cm', steering=0, speed=None):
        """
        Start the 2 motors simultaneously to move a Driving Base.
        steering=0 makes the Driving Base go straight. Negative numbers make the Driving Base turn left. Positive numbers make the Driving Base turn right.
        The program will not continue until amount is reached.
        If the value of steering is equal to -100 or 100, the Driving Base will perform a rotation on itself (tank move) with the default speed on each motor.
        If the value of steering is outside of the allowed range, the value will be set to -100 or 100 depending whether the value is positive or negative.
        If speed is outside of the allowed range, the value will be set to -100 or 100 depending whether the value is positive or negative.
        If the speed is negative, then the Driving Base will move backward instead of forward. Likewise, if amount is negative, the Driving Base will move backward instead of forward. If both the speed and the amount are negative, then the Driving Base will move forward.
        When unit is 'cm' or 'in', the amount of the unit parameter is the horizontal distance that the Driving Base will travel before stopping. The relationship between motor rotations and distance traveled can be adjusted by calling set_motor_rotation().
        When 'unit' is 'rotations' or 'degrees', the amount parameter value specifies how much the motor axle will turn before stopping.
        When unit is 'seconds', the amount parameter value specifies the amount of time the motors will run before stopping.
        
        Parameters
        -------------
        amount : The quantity to move in relation to the specified unit of measurement.
        
        Type : float (decimal numbers)
        
        Values : any value
        
        Default : no default value
        
        unit : The units of measurement of the amount parameter
        
        Type : string (text)
        
        Values : 'cm','in','rotations','degrees','seconds'
        
        Default : cm
        
        steering : The direction and the quantity to steer the Driving Base.
        
        Type : integer (positive or negative whole number, including 0)
        
        Values : -100 to 100
        
        Default : 0
        
        speed : The motor speed.
        
        Type : integer (positive or negative whole number, including 0)
        
        Values : -100 to 100
        
        Default : the speed set by set_default_speed()
        
        Errors
        --------------
        TypeError : amount is not a number, or steering or speed is not an integer, or unit is not a string.
        
        ValueError : unit is not one of the allowed values.
        
        RuntimeError : One or both of the motors has been disconnected or the motors could not be paired.
        """
        pass
    def start(self, steering=0, speed=None):
        """
        Start the 2 motors simultaneously, to will move a Driving Base.
        steering=0 makes the Driving Base go straight. Negative numbers make the Driving Base turn left. Positive numbers make the Driving Base turn right.
        The program flow is not interrupted. This is most likely interrupted by a sensor input and a condition.
        If the value of steering is equal to -100 or 100, the Driving Base will perform a rotation on itself (tank move) with the default speed on each motor.
        If the value of steering is outside of the allowed range, the value will be set to -100 or 100 depending on whether the value is positive or negative.
        If speed is outside of the allowed range, the value will be set to -100 or 100 depending whether the value is positive or negative.
        If the speed is negative, then the Driving Base will move backward instead of forward. Likewise, if amount is negative, the Driving Base will move backward instead of forward. If both the speed and the amount are negative, then the Driving Base will move forward.
        
        Parameters
        ------------
        steering : The direction and the quantity to steer the Driving Base.
        
        Type : integer (positive or negative whole number, including 0)
        
        Values : -100 to 100
        
        Default : 0
        
        speed : The speed at which the Driving Base will move while performing a curve.
        
        Type : integer (positive or negative whole number, including 0)
        
        Values : -100% to 100%
        
        Default : if no value is specified, it will use the default speed set by set_default_speed().
        
        Errors
        -----------
        TypeError : Steering or speed is not an integer.
        
        RuntimeError : One or both of the motors has been disconnected or the motors could not be paired.
        """
        pass
    def stop(self):
        """
        Stops the 2 motors simultaneously, which will stop a Driving Base.
        The motors will either actively hold their current position or coast freely depending on the option selected by set_stop_action().
        Errors
        --------------
        RuntimeError : One or both of the motors has been disconnected or the motors could not be paired.
        """
        pass
    def move_tank(self, amount, unit='cm', left_speed=None, right_speed=None):
        """
        Moves the Driving Base using differential (tank) steering.
        The speed of each motor can be controlled independently for differential (tank) drive Driving Bases.
        When unit is 'cm' or 'in', the amount of the unit parameter is the horizontal distance that the Driving Base will travel before stopping. The relationship between motor rotations and distance traveled can be adjusted by calling set_motor_rotation().
        When 'unit' is 'rotations' or 'degrees', the amount parameter value specifies how much the motor axle will turn before stopping.
        When unit is 'seconds', the amount parameter value specifies the amount of time the motors will run before stopping.
        If left_speed or right_speed is outside of the allowed range, the value will be set to -100 or 100 depending whether the value is positive or negative.
        If one of the speed is negative (left_speed or right_speed), then the motor with that negative speed will run backward instead of forward. If the value of the amount parameter is negative, both motors will rotate backward instead of forward. If both the speed values (left_speed or right_speed) are negative and the value of the amount parameter is negative, then the both motors will rotate forward.
        The program will not continue until amount is reached.
        
        Parameters
        -----------------
        amount : The quantity to move in relation to the specified unit of measurement.
        
        Type : float (decimal number)
        
        Values : any value
        
        Default : no default value
        
        unit : The units of measurement of the amount parameter
        
        Type : string (text)
        
        Values : 'cm','in','rotations','degrees','seconds'
        
        Default : cm
        
        left_speed : The speed of the left motor
        
        Type : integer (positive or negative whole number, including 0)
        
        Values : -100 to 100
        
        Default : the speed set by set_default_speed()
        
        right_speed : The speed of the right motor
        
        Type : integer (positive or negative whole number, including 0)
        
        Values : -100 to 100
        
        Default : the speed set by set_default_speed()
        
        Errors
        --------------
        TypeError : amount, left_speed or right_speed is not a number or unit is not a string.
        
        ValueError : unit is not one of the allowed values.
        
        RuntimeError : One or both of the Ports do not have a motor connected or the motors could not be paired.
        """
        pass
    def start_tank(self, left_speed, right_speed):
        """
        Starts moving the Driving Base using differential (tank) steering.
        The speed of each motor can be controlled independently for differential (tank) drive Driving Bases.
        If left_speed or right_speed is outside of the allowed range, the value will be set to -100 or 100 depending whether the value is positive or negative.
        If a speed is negative, then the motors will move backward instead of forward.
        The program flow is not interrupted. This is most likely interrupted by a sensor input and a condition.
        
        Parameters
        ----------
        left_speed : The speed of the left motor.
        
        Type : integer (positive or negative whole number, including 0)
        
        Values : -100 to 100
        
        Default : no default value
        
        right_speed : The speed of the right motor.
        
        Type : integer (positive or negative whole number, including 0)
        
        Values : -100 to 100
        Default : no default value
        
        Errors
        -------
        TypeError : left_speed or right_speed is not an integer.
        RuntimeError : One or both of the Ports do not have a motor connected or the motors could not be paired.
        """
        pass

    def start_at_power(self, power, steering=0):
        """
        Starts moving the Driving Base without speed control.
        The motors can also be driven without speed control. This is useful when using your own control algorithm (e.g., a proportional line follower).
        If steering is outside of the allowed range, the value will be set to -100 or 100 depending whether the value is positive or negative.
        If power is outside of the allowed range, the value will be set to -100 or 100 depending whether the value is positive or negative.
        If the power is negative, then Driving Base will move backward instead of forward.
        The program flow is not interrupted. This can most likely interrupted by a sensor input and a condition.
        
        Parameters
        --------------
        power :  The amount of power to send to the motors.
        
        Type : integer (positive or negative whole number, including 0)
        
        Values : -100 to 100 %
        
        Default : 100
        
        steering : The steering direction (-100 to 100). 0 makes the Driving Base go straight. Negative numbers make the Driving Base turn left. Positive numbers make the Driving Base turn right.
        
        Type : Integer
        
        Values : -100 to 100
        
        Default : 0
        
        Errors
        ----------
        TypeError : steering or power is not an integer.
        
        RuntimeError : One or both of the Ports do not have a motor connected or the motors could not be paired.
        """
        pass
    def start_tank_at_power(self, left_power, right_power):
        """
        Starts moving the Driving Base using differential (tank) steering without speed control.
        The motors can also be driven without speed control. This is useful when using your own control algorithm (e.g., a proportional line follower).
        If left_power or right_power is outside of the allowed range, the value will be rounded to -100 or 100 depending on whether the value is positive or negative.
        If a power is negative, then the corresponding motor will move backward instead of forward.
        The program flow is not interrupted. This can most likely interrupted by a sensor input and a condition.
        
        Parameters
        ===========
        left_power : The power of the left motor
       
        Type : Integer
        
        Values : -100 to 100
        
        Default : no default value
        
        right_power : The power of the right motor
        
        Type : Integer
        
        Values : -100 to 100
        
        Default : no default value
        
        Errors
        ------------
        TypeError : left_power or right_power is not an integer.
        
        RuntimeError : One or both of the Ports do not have a motor connected or the motors could not be paired.
        """
        pass
    def get_default_speed(self):
        """
        Retrieves the default motor speed.
        
        Returns
        -----------
        The default motor speed.
        
        Type : integer (positive or negative whole number, including 0)
        
        Values : -100 to 100 %
        Errors
        ------------
        RuntimeError : One or both of the Ports do not have a motor connected or the motors could not be paired.
        """
        pass
    def set_motor_rotation(self, amount, unit='cm'):
        """
        Sets the ratio of one motor rotation to the distance traveled.
        If there are no gears used between the motors and the wheels of the Driving Base, then amount is the circumference of one wheel.
        Calling this method does not affect the Driving Base if it is already currently running. It will only have an effect the next time one of the move or start methods is used.
        
        Parameters
        -----------
        amount : The distance the Driving Base moves when both motors move one rotation each.
        
        Type : float (decimal number)
        
        Values : any value
        
        Default : 17.6cm (large wheel)
        
        unit : The units of measurement of the amount parameter.
        
        Type : string (text)
        
        Values : 'cm','in'
        
        Default : cm
        
        Errors
        ------------
        TypeError : amount is not a number or unit is not a string.
        
        ValueError : unit is not one of the allowed values.
        
        RuntimeError : One or both of the Ports do not have a motor connected or the motors could not be paired.
        """
        pass
    def set_default_speed(self, speed):
        """
        Sets the default motor speed.
        If speed is outside of the allowed range, the value will be set to -100 or 100 depending on whether the value is positive or negative.
        Setting the speed will not have any effect until one of the move or start methods is called, even if the Driving Base is already moving.
        
        Parameters
        --------
        speed : The default motor speed
        
        Type : integer (positive or negative whole number, including 0)
        
        Values : -100 to 100
        
        Default : 100
        
        Errors
        ----------
        TypeError : speed is not a number.
        
        RuntimeError : One or both of the Ports do not have a motor connected or the motors could not be paired.
        """
        pass
    def set_stop_action(self, action):
        """
        Sets the motor action that will be used when the Driving Base stops.
        If action is 'brake' the motors will stop quickly but will be allowed to turn freely.
        If action is 'hold', the motors will actively hold their current position and cannot be turned manually.
        If action is set to 'coast', the motors will stop slowly and will be able to be turned freely.
        Setting the stop action does not take immediate effect on the motors. The setting will be saved and used whenever stop() is called or when one of the move methods has completed without being interrupted.
        
        Parameters
        -----------
        action : The desired action of the motors when the Driving Base stops.
        
        Type : string (text)
        
        Values : 'brake','hold','coast'
        
        Default : 'coast'
        
        Errors
        -----------
        TypeError : action is not a string.
        
        ValueError :  action is not one of the allowed values.
        RuntimeError : One or both of the Ports do not have a motor connected or the motors could not be paired.
        """
        pass

class Speaker:
    def __init__(self):
        """
        Controls this SpikeHub speaker
        """
        pass

    def beep(self, note=60, seconds=0.2):
        """
        Plays a beep on the Hub.
        Your program will not continue until seconds have passed.
        Parameters
        ---------
        note : The MIDI note number.
        Type : float (decimal number)
        Values : 44 to 123 (60 is middle C note)
        Default : 60 (middle C note)
        seconds : The duration of the beep in seconds.
        Type : float (decimal number)
        Values : any values
        Default : 0.2 seconds
        Errors
        ---------
        TypeError : note is not an integer or seconds is not a number.
        
        ValueError : note is not within the allowed range of 44-123.
        """
        pass
    def start_beep(self, note=60):
        """
        Starts playing a beep.
        The beep will play indefinitely until stop() or another beep method is called.
        
        Parameters
        -----------
        note : The MIDI note number.
        
        Type : float (decimal number)
        
        Values : 44 to 123 (60 is middle C note)
        
        Default : 60 (middle C note)
        
        Errors
        ------------
        TypeError : note is not an integer.
        
        ValueError : note is not within the allowed range of 44-123
        """
        pass
    def stop(self):
        """
        Stops any sound that is playing.
        """
        pass
    def get_volume(self):
        """
        Retrieves the value of the speaker volume.
        This only retrieves the volume for the Hub and not the programming app.
        
        Returns
        --------------
        The current volume.
        
        Type : integer (positive or negative whole number, including 0)
        
        Values : 0 to 100%
        """
        pass
    def set_volume(self, volume):
        """
        Sets the speaker volume.
        If the assigned volume is out of range, the nearest volume (0 or 100) will be used instead. This only sets the volume for the Hub and not the programming app.
        
        Parameters
        -----------
        volume : The new volume percentage.
        
        Type : integer (positive or negative whole number, including 0)
        
        Values : 0 to 100%
        
        Default : 100%
        
        Errors
        ------------
        TypeError : volume is not an integer.
        """
        pass

class StatusLight:
    def __init__(self):
        """
        Controls the Status light on the PrimeHub
        """
        pass
    def on(self, color='white'):
        """
        Sets the color of the light.
        
        Parameters
        --------
        color : Illuminates the Brick Status Light on the Hub in the specified color
        
        Type : string (text)
        
        Values : azure,black,'blue','cyan','green','orange','pink','red','violet','yellow','white'
        
        Default : 'white'
        
        Errors
        -------
        TypeError : color is not a string.
        
        ValueError : color is not one of the allowed values.
        """
        pass
    def off(self):
        """
        Turns the status light off
        """
        pass

def wait_for_seconds(seconds):
    """
    Waits for a specified number of seconds before continuing the program.
    
    Parameters
    --------------
    seconds : The time to wait in seconds.
    
    Type : float (decimal value)
    
    Values : any value
    
    Default : no default value
    
    Errors
    ----------------
    TypeError : seconds is not a number.
    
    ValueError : seconds is not at least 0.
    """
    pass
def wait_until(get_value_function, operator_function, target_value=True):
    """
    Waits until the condition is True before continuing with the program.
    
    Parameters
    --------------
    get_value_function
    
    Type : callable function
    
    Values : A function that returns the current value to be compared to the target value.
    
    Default : no default value
    
    operator_function
    
    Type : callable function
    
    Values : A function that compares two arguments. The first argument will be the result of get_value_function() and the second argument will be target_value. The function will compare these two values and return the result.
    
    Default : no default value
    
    target_value
    
    Type : any type
    
    Values : Any object that can be compared by operator_function.
    
    Default : no default value
    
    Errors
    ----------------
    TypeError : get_value_function or operator_function is not callable or operator_function does not compare two arguments.
    """
    pass

def greater_than(a, b):
    """
    Tests whether value a is greater than value b.
    This is the same as a > b.
    
    Parameters
    ---------
    a : Any object that can be compared to b.
    
    Type : any type
    
    Values : any value
    
    Default : no default value
    
    b : Any object that can be compared to a.
    
    Type : any type
    
    Values : any value
    
    Default : no default value
    
    Returns
    ---------
    Type : boolean
    
    Values : True if a > b, otherwise False.
    """
    pass


class Buttons:
    def __init__(self, location):
        """
        Accesses the PrimeHub buttons
        """
        pass
    def wait_until_pressed(self):
        """
        Waits until the button is pressed.
        """
        pass
    def wait_until_released(self):
        """
        Waits until the button is released.
        """
        pass
    def was_pressed(self):
        """
        Tests to see whether the button has been pressed since the last time this method called.
        Once this method returns True, the button must be released and pressed again before this method will return True again.
        Returns
        -------------
        If the button was pressed, otherwise
        Type : boolean
        Values : True or False
        """
        pass
    def is_pressed(self):
        """
        Tests whether the button is pressed.
        Returns
        ----------
        True if the button is pressed, otherwise False
        
        Type : boolean
        
        Values : True or False
        """
        pass

class ColorSensor:
    def __init__(self, port):
        """
        Controls the color sensor.
        Parameters
        -------------
        port : The port label the sensor is connected to.
        
        Type : string (text)
        
        Values : 'A', 'B', 'C', 'D', and 'E'
        
        Default : no default value
        """
        pass
        def get_color(self):
            """
            Retrieves the detected color of a surface.
            
            Returns
            -----------
            Name of the color.
            Type : string (text)
            Values : 'black','violet','blue','cyan','green','yellow','red','white',None
            
            Errors
            -----------
            RuntimeError : The sensor has been disconnected from the Port.
            """
        pass
    def get_ambient_light(self):
        """
        Retrieves the intensity of the ambient light.
        This causes the Color Sensor to change modes, which can affect your program in unexpected ways. For example, when the Color Sensor is in ambient light mode, it cannot read colors.
        Returns
        -------------
        The ambient light intensity.
        Type : integer (positive or negative whole number, including 0)
        
        Values : 0 to 100 %
        
        Errors
        --------------
        RuntimeError : The sensor has been disconnected from the Port.
        """
        pass
    def get_reflected_light(self):
        """
        Retrieves the intensity of the reflected light.
        
        Returns
        ---------
        The reflected light intensity.
        
        Type : integer (positive or negative whole number, including 0)
        
        Values : 0 to 100 %
        
        Errors
        ------------
        RuntimeError : The sensor has been disconnected from the Port.
        """
        pass
    def get_rgb_intensity(self):
        """
        Retrieves the red, green, blue, and overall color intensity.
        
        Returns
        ----------
        Type : tuple of int
        
        Values : Red, green, blue, and overall intensity (0-1024)
        
        Errors
        -----------
        RuntimeError : The sensor has been disconnected from the Port.
        """
        pass
    def get_red(self):
        """
        Retrieves the red color intensity.
        
        Returns
        --------
        Type : integer (positive or negative whole number, including 0)
        
        Values : 0 to 1024
        
        Errors
        ----------
        RuntimeError : The sensor has been disconnected from the Port.
        """
        pass
    def get_green(self):
        """
        Retrieves the green color intensity.
        
        Returns
        -----------
        Type : integer (positive or negative whole number, including 0)
        
        Values : 0 to 1024
        
        Errors
        -----------
        RuntimeError : The sensor has been disconnected from the Port.
        """
        pass
    def get_blue(self):
        """
        Retrieves the blue color intensity.
        
        Returns
        -----------
        Type : integer (positive or negative whole number, including 0)
        
        Values : 0 to 1024
        
        Errors
        -----------
        RuntimeError : The sensor has been disconnected from the Port.
        """
        pass
    def wait_until_color(self, color):
        """
        Waits until the Color Sensor detects the specified color.
        
        Parameters
        -----------
        color : The name of the color
        
        Type : string (text)
        
        Values : 'black','violet','blue','cyan','green','yellow','red','white',None
        
        Default : no default value
        
        Errors
        ------------
        TypeError : color is not a string or None.
        
        ValueError : color is not one of the allowed values.
        
        RuntimeError : The sensor has been disconnected from the Port.
        """
        pass
    def wait_for_new_color(self):
        """
        Waits until the Color Sensor detects a new color.
        The first time this method is called, it returns immediately the detected color. After that, it waits until the Color Sensor detects a color that is different from the color that was detected the last time this method was used.
        
        Returns
        -----------
        The name of the new color
        
        Type : string (text)
        
        Values : 'black','violet','blue','cyan','green','yellow','red','white',None
        
        Errors
        ----------
        RuntimeError : The sensor has been disconnected from the Port.
        """
        pass
    def light_up_all(self, brightness=100):
        """
        Lights up all of the lights on the Color Sensor with a specified brightness.
        This causes the Color Sensor to change modes, which can affect your program in unexpected ways. For example, when the Color Sensor is in light up mode, it cannot read colors.
        
        Parameters
        ---------
        brightness : The desired brightness of the lights on the Color Sensor.
        
        Type : integer (positive or negative whole number, including 0)
        
        Values : 0 to 100 % (0 is off and 100 is full brightness.)
        
        Default : 100 %
        
        Errors
        ---------- 
        TypeError : brightness is not an integer.
        
        RuntimeError : The sensor has been disconnected from the Port.
        """
        pass
    def light_up(self, light_1=100, light_2=100, light_3=100):
        """
        Sets the brightness of the individual lights on the Color Sensor.
        This causes the Color Sensor to change modes, which can affect your program in unexpected ways. For example, when the Color Sensor is in light up mode, it cannot read colors.
        Parameters
        -------------
        light_1 : The desired brightness of light 1.
        
        Type : integer (positive or negative whole number, including 0)
        
        Values : 0 to 100 % (0 is off and 100 is full brightness.)
        
        Default : 100 %
        
        light_2 : The desired brightness of light 2.
        
        Type : integer (positive or negative whole number, including 0)
        
        Values : 0 to 100 % (0 is off and 100 is full brightness.)
        
        Default : 100 %
        
        light_3 : The desired brightness of light 3.
        
        Type : integer (positive or negative whole number, including 0)
        
        Values : 0 to 100 % (0 is off and 100 is full brightness.)
        
        Default : 100 %
        
        Errors
        --------
        TypeError : light_1, light_2, or light_3 is not an integer.
        
        RuntimeError : The sensor has been disconnected from the Port.
        """
        pass

class PrimeHub:
    speaker = Speaker()
    motion_sensor = MotionSensor()
    left_button = Buttons("left")
    right_button = Buttons("right")
    light_matrix = LightMatrix()
    status_light = StatusLight()
    PORT_A = 'A'
    PORT_B = 'B'
    PORT_C = 'C'
    PORT_D = 'D'
    PORT_E = 'E'

    def __init__(self):
        """
        The PrimeHub is divided into six components, each with a number of functions linked to it. To be able to use the Hub, you must initialize it.
        """
        pass