#include <SPI.h>
#include <FS.h>
#include <SD_MMC.h>
#include <Wire.h>
#include <math.h>

#include "Display_EPD_W21_spi.h"
#include "Display_EPD_W21.h"

// SD card pins from the MaTouch ESP32-S3 2.13-inch V1.0 schematic.
constexpr int SD_CLK_PIN = 39;
constexpr int SD_CMD_PIN = 38;
constexpr int SD_D0_PIN = 40;

// QMI8658A is already fitted on the board: I2C on GPIO17/18 and INT1 on GPIO14.
constexpr int IMU_SDA_PIN = 17;
constexpr int IMU_SCL_PIN = 18;
constexpr int IMU_INT_PIN = 14;
constexpr uint8_t QMI8658A_WHO_AM_I = 0x05;

// QMI8658A register addresses used by this sketch.
constexpr uint8_t QMI8658A_REG_WHO_AM_I = 0x00;
constexpr uint8_t QMI8658A_REG_CTRL1 = 0x02;
constexpr uint8_t QMI8658A_REG_CTRL2 = 0x03;
constexpr uint8_t QMI8658A_REG_CTRL3 = 0x04;
constexpr uint8_t QMI8658A_REG_CTRL7 = 0x08;
constexpr uint8_t QMI8658A_REG_AX_L = 0x35;

// The accelerometer is configured for +/-8 g at 1 kHz.  A valid pose must be
// close to one gravity and remain unchanged before the EPD is refreshed.
constexpr float QMI8658A_ACCEL_G_PER_LSB = 8.0f / 32768.0f;
constexpr unsigned long POSE_SAMPLE_INTERVAL_MS = 50;
constexpr unsigned long POSE_STABLE_TIME_MS = 500;
constexpr unsigned long POSE_DIAGNOSTIC_INTERVAL_MS = 500;
constexpr float POSE_GRAVITY_MIN_G = 0.80f;
constexpr float POSE_GRAVITY_MAX_G = 1.20f;
constexpr float POSE_MATCH_MIN_DOT_PRODUCT = 0.80f;

// Image2Lcd is configured as: 250 x 128, vertical scan, 4 gray levels.
constexpr uint16_t BMP_WIDTH = 250;
constexpr uint16_t BMP_HEIGHT = 128;
constexpr uint16_t BYTES_PER_COLUMN = BMP_HEIGHT / 4; // Four 2-bit pixels/byte.

const char *const kImageFiles[] = {"/image1.bmp", "/image2.bmp", "/image3.bmp"};

struct PoseTarget {
  float gravityX;
  float gravityY;
  float gravityZ;
  uint8_t imageIndex;
  const char *name;
};

// These three vectors describe the gravity direction measured by the IMU for
// photos 1, 2 and 3.  The assumed board axes are useful starting values.  Use
// the "IMU g" serial output after flashing to adjust only these values if the
// installed IMU orientation differs from the assumption.
const PoseTarget kPoseTargets[] = {
    {0.0f, -1.0f, 0.0f, 0, "photo 1 / LeoBo"},
    {-1.0f, 0.0f, 0.0f, 1, "photo 2 / cherry"},
    {0.0f, 1.0f, 0.0f, 2, "photo 3 / Makerfabs"},
};
constexpr size_t POSE_TARGET_COUNT = sizeof(kPoseTargets) / sizeof(kPoseTargets[0]);

uint8_t imageBuffer[EPD_ARRAY];
uint8_t qmi8658aAddress = 0;

static bool readBytes(File &file, uint8_t *buffer, size_t count)
{
  return file.read(buffer, count) == count;
}

static uint16_t readLE16(const uint8_t *p)
{
  return static_cast<uint16_t>(p[0]) |
         (static_cast<uint16_t>(p[1]) << 8);
}

static uint32_t readLE32(const uint8_t *p)
{
  return static_cast<uint32_t>(p[0]) |
         (static_cast<uint32_t>(p[1]) << 8) |
         (static_cast<uint32_t>(p[2]) << 16) |
         (static_cast<uint32_t>(p[3]) << 24);
}

static bool qmi8658aWriteRegister(uint8_t reg, uint8_t value)
{
  if (qmi8658aAddress == 0) {
    return false;
  }
  Wire.beginTransmission(qmi8658aAddress);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

static bool qmi8658aReadRegisters(uint8_t reg, uint8_t *buffer, size_t count)
{
  if (qmi8658aAddress == 0) {
    return false;
  }
  Wire.beginTransmission(qmi8658aAddress);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  const size_t received = Wire.requestFrom(static_cast<uint8_t>(qmi8658aAddress),
                                           static_cast<uint8_t>(count));
  if (received != count) {
    return false;
  }

  for (size_t i = 0; i < count; ++i) {
    buffer[i] = Wire.read();
  }
  return true;
}

static bool initImu()
{
  Wire.begin(IMU_SDA_PIN, IMU_SCL_PIN);
  Wire.setClock(400000);

  // The reference hardware test searches both valid QMI8658A addresses.  The
  // schematic uses 0x6A, but an assembled board can use 0x6B if SA0 differs.
  const uint8_t addresses[] = {0x6A, 0x6B};
  for (uint8_t address : addresses) {
    qmi8658aAddress = address;
    uint8_t chipId = 0;
    if (qmi8658aReadRegisters(QMI8658A_REG_WHO_AM_I, &chipId, 1) &&
        chipId == QMI8658A_WHO_AM_I) {
      Serial.printf("QMI8658A found at I2C address 0x%02X\n", qmi8658aAddress);
      break;
    }
    qmi8658aAddress = 0;
  }
  if (qmi8658aAddress == 0) {
    Serial.println("QMI8658A not found at I2C address 0x6A or 0x6B");
    return false;
  }

  // These are the known-good settings from the board's hardware test:
  // CTRL2: +/-8 g accelerometer; CTRL3: gyroscope; CTRL7: enable both.
  if (!qmi8658aWriteRegister(QMI8658A_REG_CTRL1, 0x60) ||
      !qmi8658aWriteRegister(QMI8658A_REG_CTRL2, 0x23) ||
      !qmi8658aWriteRegister(QMI8658A_REG_CTRL3, 0x53) ||
      !qmi8658aWriteRegister(QMI8658A_REG_CTRL7, 0x03)) {
    Serial.println("QMI8658A configuration failed");
    return false;
  }

  delay(100); // Allow a cold-started IMU to produce its first sample.
  pinMode(IMU_INT_PIN, INPUT);
  Serial.println("QMI8658A ready: place the board in one of the three configured directions");
  return true;
}

static bool readAccelerometerG(float &ax, float &ay, float &az)
{
  uint8_t data[6];
  if (!qmi8658aReadRegisters(QMI8658A_REG_AX_L, data, sizeof(data))) {
    return false;
  }

  const int16_t rawX = static_cast<int16_t>(static_cast<uint16_t>(data[0]) |
                                             (static_cast<uint16_t>(data[1]) << 8));
  const int16_t rawY = static_cast<int16_t>(static_cast<uint16_t>(data[2]) |
                                             (static_cast<uint16_t>(data[3]) << 8));
  const int16_t rawZ = static_cast<int16_t>(static_cast<uint16_t>(data[4]) |
                                             (static_cast<uint16_t>(data[5]) << 8));
  ax = rawX * QMI8658A_ACCEL_G_PER_LSB;
  ay = rawY * QMI8658A_ACCEL_G_PER_LSB;
  az = rawZ * QMI8658A_ACCEL_G_PER_LSB;
  return true;
}

// Returns the selected image index, or -1 while the device is moving, tilted,
// or in a direction not assigned to a picture.
static int detectPoseImage(float ax, float ay, float az)
{
  const float magnitudeG = sqrtf(ax * ax + ay * ay + az * az);
  if (magnitudeG < POSE_GRAVITY_MIN_G || magnitudeG > POSE_GRAVITY_MAX_G) {
    return -1;
  }

  const float normalizedX = ax / magnitudeG;
  const float normalizedY = ay / magnitudeG;
  const float normalizedZ = az / magnitudeG;
  float bestDotProduct = -1.0f;
  int bestImageIndex = -1;

  for (size_t i = 0; i < POSE_TARGET_COUNT; ++i) {
    const PoseTarget &target = kPoseTargets[i];
    const float dotProduct = normalizedX * target.gravityX +
                             normalizedY * target.gravityY +
                             normalizedZ * target.gravityZ;
    if (dotProduct > bestDotProduct) {
      bestDotProduct = dotProduct;
      bestImageIndex = target.imageIndex;
    }
  }

  return bestDotProduct >= POSE_MATCH_MIN_DOT_PRODUCT ? bestImageIndex : -1;
}

// A pose is accepted only after it remains unchanged for POSE_STABLE_TIME_MS.
// This prevents an update while the board is being turned or carried.
static int confirmedPoseImage()
{
  static unsigned long lastSampleMs = 0;
  static unsigned long lastDiagnosticMs = 0;
  static unsigned long candidateSinceMs = 0;
  static int candidateImageIndex = -1;

  const unsigned long now = millis();
  if (now - lastSampleMs < POSE_SAMPLE_INTERVAL_MS) {
    return -1;
  }
  lastSampleMs = now;

  float ax, ay, az;
  if (!readAccelerometerG(ax, ay, az)) {
    candidateImageIndex = -1;
    return -1;
  }

  const int imageIndex = detectPoseImage(ax, ay, az);
  if (now - lastDiagnosticMs >= POSE_DIAGNOSTIC_INTERVAL_MS) {
    lastDiagnosticMs = now;
    if (imageIndex >= 0) {
      Serial.printf("IMU g: ax=%+.2f ay=%+.2f az=%+.2f; pose=%d\n", ax, ay, az, imageIndex + 1);
    } else {
      Serial.printf("IMU g: ax=%+.2f ay=%+.2f az=%+.2f; pose=none\n", ax, ay, az);
    }
  }

  if (imageIndex != candidateImageIndex) {
    candidateImageIndex = imageIndex;
    candidateSinceMs = now;
    return -1;
  }
  if (imageIndex < 0 || now - candidateSinceMs < POSE_STABLE_TIME_MS) {
    return -1;
  }
  return imageIndex;
}

// Convert RGB to the four source values expected by PIC_display().
// 0 = white, 1 = yellow, 2 = red, 3 = black after Color_get().
static uint8_t rgbToFourColor(uint8_t redChannel, uint8_t greenChannel, uint8_t blueChannel)
{
  const uint8_t luminance = static_cast<uint8_t>(
      (77U * redChannel + 150U * greenChannel + 29U * blueChannel) >> 8);

  // Detect the two chromatic panel colors before applying grayscale thresholds.
  // A pure yellow has high R/G and a low B component, but is bright enough to
  // otherwise be mistaken for white by a luminance-only conversion.
  if (redChannel >= 150 && greenChannel >= 120 && blueChannel <= 120 &&
      redChannel + 70 >= greenChannel) {
    return 1; // yellow
  }
  if (redChannel >= 130 && redChannel >= greenChannel + 45 &&
      redChannel >= blueChannel + 45) {
    return 2; // red
  }

  if (luminance >= 160) return 0; // white
  if (luminance < 90) return 3;   // black

  // Neutral midtones have no equivalent on the panel. Use red as the third
  // tone so photos retain more detail instead of turning entirely white.
  return 2;
}

// Load an uncompressed 250 x 128 BMP. Both 24-bit BGR and 32-bit BGRA files
// are accepted. The output layout intentionally follows Image2Lcd's vertical
// scan option, so the horizontal 250 x 128 picture is not rotated.
static bool loadBmp250x128(const char *path)
{
  File file = SD_MMC.open(path, FILE_READ);
  if (!file) {
    Serial.printf("Cannot open %s\n", path);
    return false;
  }

  uint8_t header[54];
  if (!readBytes(file, header, sizeof(header)) || header[0] != 'B' || header[1] != 'M') {
    Serial.printf("%s is not a BMP file\n", path);
    file.close();
    return false;
  }

  const uint32_t pixelOffset = readLE32(header + 10);
  const uint32_t dibSize = readLE32(header + 14);
  const int32_t width = static_cast<int32_t>(readLE32(header + 18));
  const int32_t rawHeight = static_cast<int32_t>(readLE32(header + 22));
  const uint16_t planes = readLE16(header + 26);
  const uint16_t bitsPerPixel = readLE16(header + 28);
  const uint32_t compression = readLE32(header + 30);

  const int32_t height = rawHeight < 0 ? -rawHeight : rawHeight;
  if (dibSize < 40 || width != BMP_WIDTH || height != BMP_HEIGHT || planes != 1 ||
      (bitsPerPixel != 24 && bitsPerPixel != 32) || compression != 0) {
    Serial.printf("Unsupported BMP: %ld x %ld, %u bpp, compression %lu\n",
                  static_cast<long>(width), static_cast<long>(rawHeight),
                  bitsPerPixel, static_cast<unsigned long>(compression));
    file.close();
    return false;
  }

  const uint8_t bytesPerPixel = bitsPerPixel / 8;
  const uint32_t rowStride = ((static_cast<uint32_t>(width) * bitsPerPixel + 31U) / 32U) * 4U;
  uint8_t row[1000]; // 250 pixels x 4 bytes; enough for 24-bit padded rows too.
  memset(imageBuffer, 0, sizeof(imageBuffer));

  for (uint16_t sourceY = 0; sourceY < BMP_HEIGHT; ++sourceY) {
    // Positive BMP heights are stored bottom-to-top. Convert only the file row
    // order here; it does not rotate or mirror the intended image.
    const uint16_t fileY = rawHeight > 0 ? BMP_HEIGHT - 1 - sourceY : sourceY;
    if (!file.seek(pixelOffset + static_cast<uint32_t>(fileY) * rowStride) ||
        !readBytes(file, row, rowStride)) {
      Serial.printf("Read failed at BMP row %u\n", sourceY);
      file.close();
      return false;
    }

    for (uint16_t sourceX = 0; sourceX < BMP_WIDTH; ++sourceX) {
      const uint8_t *pixel = row + sourceX * bytesPerPixel; // BMP stores B, G, R, [A].
      const uint8_t color = rgbToFourColor(pixel[2], pixel[1], pixel[0]);

      // Image2Lcd vertical scan: finish one 128-pixel column before the next.
      const size_t index = static_cast<size_t>(sourceX) * BYTES_PER_COLUMN + sourceY / 4;
      const uint8_t shift = 6 - 2 * (sourceY & 0x03);
      imageBuffer[index] |= color << shift;
    }
  }

  file.close();
  return true;
}

static void showImage(const char *path)
{
  Serial.printf("Loading %s\n", path);
  if (!loadBmp250x128(path)) {
    return;
  }

  EPD_init();
  PIC_display(imageBuffer);
  EPD_sleep();
  Serial.println("Display refreshed");
}

void setup()
{
  Serial.begin(115200);
  delay(200);

  pinMode(EPD_BUSY_PIN, INPUT);
  pinMode(EPD_RST_PIN, OUTPUT);
  pinMode(EPD_DC_PIN, OUTPUT);
  pinMode(EPD_CS_PIN, OUTPUT);
  SPI.begin(EPD_SCK_PIN, -1, EPD_MOSI_PIN, EPD_CS_PIN);
  SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));

  SD_MMC.setPins(SD_CLK_PIN, SD_CMD_PIN, SD_D0_PIN);
  // The final false prevents SD_MMC from formatting a card when mount fails.
  if (!SD_MMC.begin("/sdcard", true, false)) {
    Serial.println("SD card mount failed");
    while (true) delay(1000);
  }

  Serial.printf("SD card: %llu MB\n", SD_MMC.cardSize() / (1024ULL * 1024ULL));

  if (!initImu()) {
    Serial.println("IMU unavailable: check that U7 is fitted and its I2C lines are intact");
    while (true) delay(1000);
  }
}

void loop()
{
  static int displayedImageIndex = -1;

  const int poseImageIndex = confirmedPoseImage();
  if (poseImageIndex >= 0 && poseImageIndex != displayedImageIndex) {
    Serial.printf("Pose confirmed: %s\n", kPoseTargets[poseImageIndex].name);
    showImage(kImageFiles[poseImageIndex]);
    displayedImageIndex = poseImageIndex;
  }
}
