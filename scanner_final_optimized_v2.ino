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

// Vacuum Pump Relay Pin
// ACTIVE LOW: LOW = Vacuum ON (holding), HIGH = Vacuum OFF (released)
const int vacuumRelayPin     = 19;

// ── BAD SCAN INDICATOR LED ────────────────────────────────────────────────────
// Lights up when a notebook scan session ends with a page count != EXPECTED_PAGES.
// This means either a double-lift (count < 16) or a missed-lift (count > 16) occurred.
// LED OFF = good scan (exactly 16 pages)
// LED ON  = bad scan  (any other count)
// Automatically resets (OFF) when the next notebook is detected and scanning begins.
const int ledPin             = 32;  // ACTIVE HIGH: HIGH = LED ON, LOW = LED OFF
// ─────────────────────────────────────────────────────────────────────────────

// Jetson Nano UART Pins
const int RXD2 = 16;
const int TXD2 = 17;


// ==========================================
// 2. MOTION PARAMETERS (Ramped Stepping)
// ==========================================
// All intervals are microseconds BETWEEN steps. Bigger = slower.

const int  pulseWidth        = 3;     // Microseconds PUL stays HIGH

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

// ── SCAN QUALITY CONSTANTS ────────────────────────────────────────────────────
// Expected number of pages in a fully scanned notebook.
// Change this value here if notebook size changes — no other code needs editing.
const int  EXPECTED_PAGES    = 16;

// Vacuum turns OFF after this page number during scanning.
const int  VACUUM_OFF_PAGE   = 14;
// ─────────────────────────────────────────────────────────────────────────────

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
  delay(50);


  // ── Step 1: Extend pneumatic stroke ───────────────────────────────────
  Serial.println("Step 1: Extending pneumatic stroke");
  digitalWrite(pneumaticRelayPin, HIGH);
  delay(300);


  // ── Step 3: Retract pneumatic stroke to lift paper ────────────────────
  Serial.println("Step 3: Retracting pneumatic stroke");
  digitalWrite(pneumaticRelayPin, LOW);
  delay(400);

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
  Serial.print("  Shaft sensor triggered. Creeping ");
  Serial.print(HOME_OFFSET_STEPS);
  Serial.println(" more steps for 15° offset correction...");

  for (int i = 0; i < HOME_OFFSET_STEPS; i++) {
    stepOnce(HOME_INTERVAL);
    stepCount++;
  }

  Serial.println("  True home reached.");

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
  delay(100);

  // ── Step 4b: Spine press — flatten notebook spine ─────────────────────
  Serial.println("Step 4b: Spine press DOWN (flattening spine)...");
  digitalWrite(spinePressRelayPin, HIGH);
  delay(200);
  digitalWrite(spinePressRelayPin, LOW);
  Serial.println("Step 4b: Spine press UP (released).");
  delay(100);

  // ── Step 5: Increment cycle counter, then trigger camera capture ──────
  // pageCount is now the 1-based scan-cycle number for THIS notebook.
  // We send it along with the capture command as "CAPTURE:<cycle>" so the
  // Jetson can name the images by real page number:
  //     cam1 → page (2×cycle − 1)   [odd page:  1, 3, 5, ...]
  //     cam0 → page (2×cycle)       [even page: 2, 4, 6, ...]
  // Because pageCount resets to 0 when the conveyor resumes, the Jetson
  // seeing "CAPTURE:1" also means: NEW NOTEBOOK → start a fresh folder.
  pageCount++;

  Serial.print("Step 5: Sending CAPTURE:");
  Serial.print(pageCount);
  Serial.println(" to Jetson...");
  Serial2.print("CAPTURE:");
  Serial2.println(pageCount);
  delay(200);

  // ── Step 6: Log cycle completion ──────────────────────────────────────
  Serial.print("Step 6: Scan cycle complete. Pages scanned this session: ");
  Serial.println(pageCount);

  // ── Vacuum control: turn OFF after page 14 ────────────────────────────
  if (pageCount == VACUUM_OFF_PAGE) {
    digitalWrite(vacuumRelayPin, HIGH);   // Vacuum OFF
    Serial.println(">>> Page 14 reached: Vacuum OFF.");
  }
  Serial.println();
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
  pinMode(ledPin,              OUTPUT);
  pinMode(stepPin,             OUTPUT);
  pinMode(dirPin,              OUTPUT);

  // Safe initial states
  digitalWrite(conveyorRelayPin,   HIGH);  // Conveyor OFF          (Active LOW)
  digitalWrite(suctionRelayPin,    HIGH);  // Suction OFF           (Active LOW)
  digitalWrite(pneumaticRelayPin,  LOW);   // Page-turn retracted   (Active HIGH)
  digitalWrite(spinePressRelayPin, LOW);   // Spine press retracted (Active HIGH)
  digitalWrite(vacuumRelayPin,     LOW);   // Vacuum ON at startup  (Active LOW)
  digitalWrite(ledPin,             LOW);   // LED OFF at startup
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

    if (isConveyorMoving) {
      // ── Reset LED when new notebook scan begins ────────────────────────
      // Clears any bad-scan indicator from the previous notebook so the
      // operator gets a clean reading for this session.
      digitalWrite(ledPin, LOW);
      // ──────────────────────────────────────────────────────────────────
      Serial.println("Notebook + pages detected! Conveyor STOPPED. Starting scan...");
      isConveyorMoving = false;
    }

    runScanCycle();

  } else {
    // No notebook, or last page done → run conveyor
    digitalWrite(conveyorRelayPin, LOW);    // Conveyor ON

    if (!isConveyorMoving) {

      // ── Scan quality check: compare pageCount to expected 16 ──────────
      // Runs once per session, right when the conveyor resumes.
      // pageCount < 16 → double-lift occurred somewhere (pages missing)
      // pageCount > 16 → missed-lift occurred somewhere (extra cycles ran)
      // pageCount == 16 → perfect scan
      if (pageCount > 0) {
        if (pageCount != EXPECTED_PAGES) {
          digitalWrite(ledPin, HIGH);   // LED ON: bad scan
          Serial.print(">>> BAD SCAN: Expected ");
          Serial.print(EXPECTED_PAGES);
          Serial.print(" pages but scanned ");
          Serial.print(pageCount);
          Serial.println(". LED indicator ON.");
        } else {
          digitalWrite(ledPin, LOW);    // LED OFF: good scan
          Serial.println(">>> GOOD SCAN: Exactly 16 pages scanned correctly.");
        }

        Serial.print("Session total: ");
        Serial.print(pageCount);
        Serial.println(" page(s) scanned. Resetting counter for next notebook.");
        pageCount = 0;
      }
      // ──────────────────────────────────────────────────────────────────

      // Vacuum always ON when conveyor starts (ready for next notebook)
      digitalWrite(vacuumRelayPin, LOW);
      Serial.println("Vacuum ON: ready to hold next notebook.");

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
