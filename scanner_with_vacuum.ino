// ==========================================
// 1. PIN DEFINITIONS
// ==========================================

// Conveyor & Notebook Sensor Pins
const int notebookSensorPin  = 4;   // Detects the notebook on the conveyor (HIGH = detected)
const int conveyorRelayPin   = 5;   // Conveyor Motor (ACTIVE LOW: LOW = ON, HIGH = OFF)

// Scanner & Stepper Pins
const int stepPin            = 13;
const int dirPin             = 14;
const int suctionRelayPin    = 26;  // Suction pump  (ACTIVE LOW: LOW = suction ON, HIGH = suction OFF)
const int pneumaticRelayPin  = 27;  // Page-turn pneumatic (ACTIVE HIGH: HIGH = extended, LOW = retracted)

// Spine Press Pneumatic Relay Pin
// GPIO 25: safe general-purpose output, unused, away from UART2 (16/17)
// ACTIVE HIGH: HIGH = press down (extend), LOW = release (retract)
const int spinePressRelayPin = 25;

// Shaft Proximity Sensor Pin
const int shaftSensorPin     = 15;

// Last-Page Proximity Sensor Pin
// HIGH = page detected (more pages remain)
// LOW  = no page detected (last page scanned → scan complete)
const int lastPageSensorPin  = 18;

// ── VACUUM PUMP RELAY ─────────────────────────────────────────────────────────
// Holds the notebook flat on the conveyor bed during scanning.
// ACTIVE LOW: LOW = Vacuum ON (holding), HIGH = Vacuum OFF (released)
// Logic:
//   - ON  when conveyor is running (holds incoming notebook)
//   - ON  while scanning pages 1–13
//   - OFF from page 14 onwards (notebook spine end is lighter; vacuum not needed)
//   - ON  again when conveyor resumes after scan session
const int vacuumRelayPin     = 19;

// Page number at which vacuum is switched OFF during scanning.
// Pages before this: vacuum ON. Page 14 and after: vacuum OFF.
const int VACUUM_OFF_PAGE    = 14;
// ─────────────────────────────────────────────────────────────────────────────

// Jetson Nano UART Pins
const int RXD2 = 16;
const int TXD2 = 17;


// ==========================================
// 2. MOTION PARAMETERS (Ramped Stepping)
// ==========================================
// All intervals are microseconds BETWEEN steps. Bigger = slower.

const int  pulseWidth        = 5;     // Microseconds PUL stays HIGH

const int  START_INTERVAL    = 3000;  // Speed at the very first step (very slow)
const int  GRAB_INTERVAL     = 1800;  // Top speed WHILE holding the paper
const int  FREE_INTERVAL     = 800;   // Top speed AFTER paper is released
const int  HOME_INTERVAL     = 2500;  // Speed when creeping onto the home sensor

const int  ACCEL_US_PER_STEP = 6;     // µs the interval shrinks each step (acceleration)
const int  DECEL_US_PER_STEP = 6;     // µs the interval grows each step (deceleration)

// Drop the paper after this many steps even while motor is still turning
const long DROP_STEP         = 166;

// How many steps before home to start slowing down.
// Learned automatically after the first full rotation (see lastRevSteps).
const long DECEL_STEPS       = 120;

// After the shaft sensor triggers, creep this many more steps at HOME_INTERVAL
// to correct the 15° physical offset between sensor position and true home.
// Formula: (15 / 360) × steps_per_revolution
//
//   200  steps/rev (1x  microstep) →   8 steps
//   400  steps/rev (2x  microstep) →  17 steps
//   800  steps/rev (4x  microstep) →  33 steps
//   1600 steps/rev (8x  microstep) →  67 steps
//   3200 steps/rev (16x microstep) → 133 steps
//
// ↓ SET THIS to match your DM542E DIP-switch microstep setting ↓
const int  HOME_OFFSET_STEPS = 33;

// Spine press dwell time (ms) between downstroke and upstroke
const int  SPINE_PRESS_HOLD_MS = 500;

// Steps counted on the previous full rotation (0 = unknown yet)
long lastRevSteps = 0;

// State tracker to prevent Serial Monitor spam
bool isConveyorMoving = false;

// Page counter — how many scan cycles completed in the current notebook session.
// Incremented at the end of each runScanCycle().
// Reset to 0 when the conveyor starts running again.
int pageCount = 0;


// ==========================================
// 3. STEPPER HELPERS
// ==========================================
void stepOnce(int interval) {
  digitalWrite(stepPin, HIGH);
  delayMicroseconds(pulseWidth);
  digitalWrite(stepPin, LOW);
  delayMicroseconds(interval);
}

// Move interval one notch toward target (never overshoot)
int rampToward(int interval, int target, int rate) {
  if (interval > target) {
    interval -= rate;
    if (interval < target) interval = target;
  } else if (interval < target) {
    interval += rate;
    if (interval > target) interval = target;
  }
  return interval;
}


// ==========================================
// 4. SCANNING SEQUENCE FUNCTION
// ==========================================
void runScanCycle() {
  Serial.println("\n--- NEW PAGE SCAN CYCLE STARTING ---");

  // ── Step 2: Suction ON to grab paper ──────────────────────────────────
  Serial.println("Step 2: Suction ON");
  digitalWrite(suctionRelayPin, LOW);
  delay(2000);


  // ── Step 1: Extend pneumatic stroke ───────────────────────────────────
  Serial.println("Step 1: Extending pneumatic stroke");
  digitalWrite(pneumaticRelayPin, HIGH);
  delay(500);


  // ── Step 3: Retract pneumatic stroke to lift paper ────────────────────
  Serial.println("Step 3: Retracting pneumatic stroke");
  digitalWrite(pneumaticRelayPin, LOW);
  delay(500);

  // ── Step 4: Turn page with ramped speed profile ───────────────────────
  Serial.println("Step 4: Rotating (ramped) until shaft sensor triggered...");

  long stepCount    = 0;
  bool paperDropped = false;
  int  interval     = START_INTERVAL;

  // SAFETY: If sensor is already HIGH, creep forward slowly to clear it
  while (digitalRead(shaftSensorPin) == HIGH) {
    stepOnce(HOME_INTERVAL);
  }

  // Main rotation: step while sensor is LOW (not yet at home)
  while (digitalRead(shaftSensorPin) == LOW) {

    // Decide target speed for this step
    bool nearHome = (lastRevSteps > 0) && (stepCount >= lastRevSteps - DECEL_STEPS);

    int target;
    if (nearHome) {
      target = HOME_INTERVAL;   // ease into home sensor
    } else if (!paperDropped) {
      target = GRAB_INTERVAL;   // gentle while holding paper
    } else {
      target = FREE_INTERVAL;   // faster once paper is released
    }

    // Ramp toward target (accelerate or decelerate, never jump)
    int rate = (interval > target) ? ACCEL_US_PER_STEP : DECEL_US_PER_STEP;
    interval = rampToward(interval, target, rate);

    stepOnce(interval);
    stepCount++;

    // Mid-rotation drop: release suction after reaching DROP_STEP
    if (stepCount == DROP_STEP && !paperDropped) {
      digitalWrite(suctionRelayPin, HIGH);
      Serial.println("  Drop: Suction OFF → paper releases midway");
      paperDropped = true;
    }
  }

  // ── 15° offset correction ─────────────────────────────────────────────
  // Sensor has fired. Motor is already crawling at HOME_INTERVAL from the
  // nearHome deceleration above, so there is no jerk. Creep the remaining
  // HOME_OFFSET_STEPS to land exactly on true home position.
  Serial.print("  Shaft sensor triggered. Creeping ");
  Serial.print(HOME_OFFSET_STEPS);
  Serial.println(" more steps for 15° offset correction...");

  for (int i = 0; i < HOME_OFFSET_STEPS; i++) {
    stepOnce(HOME_INTERVAL);
    stepCount++;           // keep total accurate for next cycle's decel window
  }

  Serial.println("  True home reached.");
  // ─────────────────────────────────────────────────────────────────────

  // Save full revolution length (including offset) for next cycle
  lastRevSteps = stepCount;
  Serial.print("  Revolution steps recorded: ");
  Serial.println(lastRevSteps);

  // Ensure suction is off if paper wasn't dropped mid-rotation
  if (!paperDropped) {
    digitalWrite(suctionRelayPin, HIGH);
    Serial.println("  Drop: Suction OFF → paper releases at home stop");
  }

  // Let shaft physically settle before spine press
  delay(150);

  // ── Step 4b: Spine press — flatten notebook spine ─────────────────────
  // Runs AFTER the page-turn arm has reached true home,
  // BEFORE the camera capture, so the page lies flat in the scan window.
  Serial.println("Step 4b: Spine press DOWN (flattening spine)...");
  digitalWrite(spinePressRelayPin, HIGH);    // Extend: press down on spine
  delay(SPINE_PRESS_HOLD_MS);               // Hold for 500 ms
  digitalWrite(spinePressRelayPin, LOW);     // Retract: release spine
  Serial.println("Step 4b: Spine press UP (released).");
  delay(200);                               // Brief settle before capture
  // ─────────────────────────────────────────────────────────────────────

  // ── Step 5: Trigger camera capture via Jetson Nano ────────────────────
  Serial.println("Step 5: Sending CAPTURE command to Jetson...");
  Serial2.println("CAPTURE");

  // Wait for Jetson to finish before returning to main loop
  delay(1000);

  // ── Step 6: Increment page counter ────────────────────────────────────
  pageCount++;
  Serial.print("Step 6: Scan cycle complete. Pages scanned this session: ");
  Serial.println(pageCount);

  // ── Vacuum control: turn OFF after page 14 ────────────────────────────
  // Once the notebook has been scanned past page 14, the remaining pages
  // are near the spine end where the vacuum no longer provides useful hold.
  // Turn it OFF here and leave it OFF until the next notebook session.
  if (pageCount == VACUUM_OFF_PAGE) {
    digitalWrite(vacuumRelayPin, HIGH);   // Vacuum OFF (ACTIVE LOW → HIGH = OFF)
    Serial.println(">>> Page 14 reached: Vacuum OFF (stays OFF until next session).");
  }
  Serial.println();
  // ─────────────────────────────────────────────────────────────────────
}


// ==========================================
// 5. SETUP
// ==========================================
void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);

  Serial.println("System starting up...");

  // Inputs
  pinMode(notebookSensorPin, INPUT);
  pinMode(shaftSensorPin,    INPUT_PULLUP);
  pinMode(lastPageSensorPin, INPUT);

  // Outputs
  pinMode(conveyorRelayPin,    OUTPUT);
  pinMode(suctionRelayPin,     OUTPUT);
  pinMode(pneumaticRelayPin,   OUTPUT);
  pinMode(spinePressRelayPin,  OUTPUT);
  pinMode(vacuumRelayPin,      OUTPUT);
  pinMode(stepPin,             OUTPUT);
  pinMode(dirPin,              OUTPUT);

  // Safe initial states
  digitalWrite(conveyorRelayPin,   HIGH);  // Conveyor OFF          (Active LOW)
  digitalWrite(suctionRelayPin,    HIGH);  // Suction OFF           (Active LOW)
  digitalWrite(pneumaticRelayPin,  LOW);   // Page-turn retracted   (Active HIGH)
  digitalWrite(spinePressRelayPin, LOW);   // Spine press retracted (Active HIGH)
  digitalWrite(vacuumRelayPin,     LOW);   // Vacuum ON at startup  (Active LOW → LOW = ON)
  digitalWrite(stepPin,            LOW);
  digitalWrite(dirPin,             HIGH);

  Serial.println("Setup complete. Vacuum ON. System Ready.");
  delay(1000);
}


// ==========================================
// 6. MAIN LOOP
// ==========================================
void loop() {
  int notebookState = digitalRead(notebookSensorPin);
  int lastPageState = digitalRead(lastPageSensorPin);

  if (notebookState == HIGH && lastPageState == HIGH) {
    // Notebook present AND pages remain → stop conveyor, scan next page
    digitalWrite(conveyorRelayPin, HIGH);   // Conveyor OFF
    // Vacuum remains in whatever state it is currently
    // (ON for pages 1-13, OFF for page 14 onwards — managed inside runScanCycle)

    if (isConveyorMoving) {
      Serial.println("Notebook + pages detected! Conveyor STOPPED. Starting scan...");
      isConveyorMoving = false;
    }

    runScanCycle();

  } else {
    // No notebook, or last page done → run conveyor
    digitalWrite(conveyorRelayPin, LOW);    // Conveyor ON

    if (!isConveyorMoving) {

      // ── Reset page counter and restore vacuum when conveyor resumes ────
      if (pageCount > 0) {
        Serial.print("Session total: ");
        Serial.print(pageCount);
        Serial.println(" page(s) scanned. Resetting counter for next notebook.");
        pageCount = 0;
      }

      // Always turn vacuum ON when conveyor starts running.
      // This covers two cases:
      //   a) Normal end of session (vacuum was OFF since page 14) → restore it
      //   b) Short notebook (<14 pages, vacuum was still ON) → keep it ON harmlessly
      digitalWrite(vacuumRelayPin, LOW);    // Vacuum ON (Active LOW)
      Serial.println("Vacuum ON: ready to hold next notebook.");
      // ──────────────────────────────────────────────────────────────────

      if (notebookState == HIGH && lastPageState == LOW) {
        Serial.println("Scan complete. Notebook moving out. Conveyor RUNNING.");
      } else {
        Serial.println("Path clear. Conveyor RUNNING.");
      }
      isConveyorMoving = true;
    }

    delay(100);
  }
}
