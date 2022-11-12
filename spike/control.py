def wait_for_seconds(seconds):
    """
    Waits for a specified number of seconds before continuing the program.

    Parameters
    -----------
    seconds -  The time to wait, specified in seconds.

    Type : float (decimal value)

    Values : any value. Default , no default value

    Errors
    -----------
    TypeError : seconds is not a number.

    ValueError : seconds is not at least 0.

    Example
    ----------
    from spike.control import wait_for_seconds

    #wait for 3 seconds (pause the program flow)

    wait_for_seconds(3)

    """
    pass

def wait_until(get_value_function, operator_function, target_value=True):
    """
    Waits until the condition is true before continuing with the program.

    Parameters
    ----------
    get_value_function

    Type : callable function

    Values : A function that returns the current value to be compared to the target value.

    Default : no default value

    operator_function

    Type : callable function
    
    Values : A function that compares two arguments. The first argument will be the result of get_value_function(), and the second argument will be target_value. The function will compare both values and return the result.

    Default : no default value

    target_value

    Type : any type

    Values : Any object that can be compared by operator_function.

    Default : no default value

    Errors
    ----------
    TypeError : get_value_function or operator_function is not callable or operator_function does not compare two arguments.

    Example
    ----------
    from spike import ColorSensor

    from spike.control import wait_until

    from spike.operator import equal_to

    color_sensor = ColorSensor('A')


    # Wait for the Color Sensor to detect "red"

    wait_until(color_sensor.get_color, equal_to, 'red')

    Example

    from spike import ColorSensor, Motor

    from spike.control import wait_until

    color_sensor = ColorSensor('A')

    motor = Motor('B')

    def red_or_position():

        return color_sensor.get_color() == 'red' or motor.get_position() > 90

    wait_until(red_or_position)

    """
    pass

class Timer:
    """
    To use the Timer, you must first initialize it.

    Example
    -----------
    
    from spike.control import Timer
    
    # InitialiZe the Timer

    timer = Timer()

    Following are all of the functions that are linked to the Timer.

    """

    def reset(self):
        """
        Sets the Timer to "0."

        Example
        -----------
        from spike.control import Timer
    
        timer = Timer()

        # After some time...

        timer.reset()
        """
        pass

    def now(self):
        """
        Retrieves the "right now" time of the Timer.

        Returns
        -----------
        The current time, specified in seconds.

        Type : Integer (a positive or negative whole number, including 0)

        Values : A value greather than 0

        Example
        -----------
        from spike.control import Timer

        timer = Timer()
        
        while True:

            # If it has been more than 5 seconds since the Timer started

            if timer.now() > 5:

                # then break out of the while loop

                break
        """
        pass