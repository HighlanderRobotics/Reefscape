# OCR Gyro Issue Postmortem

This writeup pertains to the gyro issue which cost us quals 66 and 77 and OCR.
Match videos for those quals are available on TBA and logs were sent in a zip file on the **#software** channel on slack.

## Symptoms

The issue was first noticed in the week (ish) before OCR as a ```Missing Gyro Data``` alert.
This alert would appear at startup and never clear.
Data from the gyro would not appear in the ```Async Odo``` inputs.
The gyro inputs would have an `isConnected` value of `false` and the position and velocity inputs would not change.
During debugging we logged the `StatusCode` of the `yaw` status signal in `GyroIOPigeon2`.
This would return a value of `CanMessageStale` when the issue occurred, and `Ok` otherwise.
Once we saw a value of `RxTimeout`, but this did not reproduce and is likely another manifestation of the same issue.

## Theories

Our initial theory for the cause of the issue was a bug in the pose estimation stack.
Because of the rewrite this offseason, we weren't 100% confident in it.
Thorough inspection did not yield any issues and logging of the `StatusCode` of the `yaw` signal indicated an issue elsewhere.

At this point we had not noticed that the issue only appeared on startup, and did not investigate it as such.

The next major theory was that there was a hardware issue somewhere on the robot.
A CAN bus break seemed unlikely because the Pigeon was the first device on the bus, so we should see the entire bus go down.
Similar reasoning was applied to a CANivore issue.

This lead us to believe there was an internal hardware issue with the Pigeon.
This made sense for an intermittent issue, and given that we did not have a clear smoking gun in our code or wiring made sense to us.
However, swapping the Pigeon would be somewhat difficult and require equipment and spares we did not have stocked at the Loom.
Therefore, we did not conduct the swap.
Without any further evidence or realizing that the issue was only at startup, we did not come up with any other theories before OCR.

The issue reappeared in quals 66 at OCR, and the prevailing "internal hardware issue" theory was applied.
After the Pigeon was swapped, the robot was powered on once and the issue did not reappear.
The issue appeared on the field the next match.

Around this time at OCR, it was realized that the issue was exclusively on startup.
Similarly, it was found that the Pigeon was the first CAN device initialized.
This lead to the theory that there was a race condition involving the CANivore initialization, possible related to our use of the ```"*"``` wildcard identifier for CANivores.

Restarting the RoboRIO appeared to clear the issue, leading further credibility to a startup issue.
For the remainder of OCR, a status indicator was added to the dashboard and the drive team was instructed to restart the robot whenever the issue reappeared.

After OCR we reproduced the issue on an elevator TalonFX by initializing that device first.
This appears to confirm the race condition theory.
A fix explicitly blocking on CANivore initialization before any devices are initialized appears to work at time of writing after several reboots to attempt to reproduce the issue.

## Process failures

Several process failures resulted in this issue costing us quals 66 and 77.
In roughly chronological order, they are:

 1. Not resolving the issue after it appeared.
    This issue was clearly problematic as soon as it appeared.
    It should not have been ignored and the assumed fix should have been checked.
 2. Not rigorously testing the "fix" after applying it.
    We only powered on once after swapping the pigeon.
    The issue was known to be intermittent, therefore we should have tested it many times to ensure it would not reappear.
 3. Not checking for the failure on the field after "fixing" it.
    It was easy to add a widget to elastic to display the gyro's status.
    We even did so for elims.
    We did not do so immediately after making the fix, which resulted in the issue going unnoticed during quals 77.
