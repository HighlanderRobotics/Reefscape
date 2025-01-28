# Controller Bindings

## Driver

| Button | State | Action |
| ------ | ----- | ------ |
| Left Trigger | IDLE | Intake algae with current request |
| Right Trigger | READY_CORAL | Score coral on the selected level |
| Right Trigger | READY_ALGAE | Score algae in the selected target |
| Left Bumper | READY_CORAL | Align with a left handed branch |
| Right Bumper | READY_CORAL | Align with a right handed branch |
| Either Bumper | READY_ALGAE | Align with net |
| Either Bumper | INTAKE_ALGAE_{Reef} | Align with algae on the reef |
| Either Bumper | INTAKE_CORAL_HP | Align with closest hp slot |
| A | any | ANTI_JAM |
| B | READY_CORAL | SPIT_CORAL |
| B | READY_ALGAE | SPIT_ALGAE |
| X + Any DPAD for 0.5s | READY_{any}, IDLE | Preclimb |
| Y for 0.5s | PRE_CLIMB, CLIMB | Cancel climb (dangerous!) |
| Start + Back | any | Rezero swerve |

## Operator

| Button | Action |
| ------ | ------ |
| A | Set target coral level to L1, set target Algae intake to ground |
| X | Set target coral level to L2, set target Algae intake to low reef |
| B | Set target coral level to L3, set target Algae intake to high reef |
| Y | Set target coral level to L4, set target Algae intake to stack |
| Left Trigger | Set algae target to processor |
| Right Trigger | Set algae target to net |
| Start | Rezero elevator and arms |
