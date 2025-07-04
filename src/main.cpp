#include "parameters_and_states.h"
#include "pose_controller.h"
#include "state_controller.h"
#include "walk_controller.h"
#include <Arduino.h>

// Declaración de funciones
void initializeHexapod();
void processMovementCommands();
void executeWalkingGait();
void handlePoseControl();
void updateLegStates();
void demonstrateGaitSequence();
void demonstratePoseSequence();
void executeComplexMovement();
void emergencyStop();

// Variables globales
StateController *hexapodController = nullptr;
unsigned long lastUpdateTime = 0;
const unsigned long UPDATE_INTERVAL_MS = 20; // 50Hz control loop

// Variables de control de movimiento
float linearVelocityX = 0.0;
float linearVelocityY = 0.0;
float angularVelocity = 0.0;

// Variables de pose
float bodyRoll = 0.0;
float bodyPitch = 0.0;
float bodyYaw = 0.0;
float bodyHeight = 0.0;

void setup() {
    Serial.begin(115200);
    Serial.println("Iniciando control de robot hexápodo...");

    // Inicializar el controlador del hexápodo
    initializeHexapod();

    Serial.println("Robot hexápodo inicializado correctamente");
}

void loop() {
    unsigned long currentTime = millis();

    // Ejecutar bucle de control a 50Hz
    if (currentTime - lastUpdateTime >= UPDATE_INTERVAL_MS) {
        lastUpdateTime = currentTime;

        // Procesar comandos de movimiento
        processMovementCommands();

        // Ejecutar patrón de marcha
        executeWalkingGait();

        // Controlar pose del cuerpo
        handlePoseControl();

        // Actualizar estados de las patas
        updateLegStates();

        // Ejecutar bucle principal del controlador
        if (hexapodController) {
            hexapodController->loop();
        }
    }

    // Manejar comunicación serial para comandos
    if (Serial.available() > 0) {
        String command = Serial.readStringUntil('\n');
        command.trim();

        // Procesar comandos básicos
        if (command.startsWith("move")) {
            // Formato: "move x y angular" (ej: "move 0.5 0.0 0.0")
            int firstSpace = command.indexOf(' ', 5);
            int secondSpace = command.indexOf(' ', firstSpace + 1);

            if (firstSpace > 0 && secondSpace > 0) {
                linearVelocityX = command.substring(5, firstSpace).toFloat();
                linearVelocityY = command.substring(firstSpace + 1, secondSpace).toFloat();
                angularVelocity = command.substring(secondSpace + 1).toFloat();

                Serial.printf("Comando de movimiento: x=%.2f, y=%.2f, angular=%.2f\n",
                              linearVelocityX, linearVelocityY, angularVelocity);
            }
        } else if (command.startsWith("pose")) {
            // Formato: "pose roll pitch yaw height" (ej: "pose 0.0 0.1 0.0 0.15")
            int spaces[3];
            int spaceCount = 0;
            int pos = 5;

            while (spaceCount < 3 && pos < command.length()) {
                pos = command.indexOf(' ', pos);
                if (pos > 0) {
                    spaces[spaceCount++] = pos;
                    pos++;
                }
            }

            if (spaceCount == 3) {
                bodyRoll = command.substring(5, spaces[0]).toFloat();
                bodyPitch = command.substring(spaces[0] + 1, spaces[1]).toFloat();
                bodyYaw = command.substring(spaces[1] + 1, spaces[2]).toFloat();
                bodyHeight = command.substring(spaces[2] + 1).toFloat();

                Serial.printf("Comando de pose: roll=%.3f, pitch=%.3f, yaw=%.3f, height=%.3f\n",
                              bodyRoll, bodyPitch, bodyYaw, bodyHeight);
            }
        } else if (command == "stop") {
            linearVelocityX = 0.0;
            linearVelocityY = 0.0;
            angularVelocity = 0.0;
            Serial.println("Deteniendo movimiento");
        } else if (command == "status") {
            if (hexapodController) {
                Serial.printf("Estado del sistema: %d\n", hexapodController->getSystemState());
                Serial.printf("Velocidad linear: x=%.2f, y=%.2f\n", linearVelocityX, linearVelocityY);
                Serial.printf("Velocidad angular: %.2f\n", angularVelocity);
                Serial.printf("Pose: roll=%.3f, pitch=%.3f, yaw=%.3f, height=%.3f\n",
                              bodyRoll, bodyPitch, bodyYaw, bodyHeight);
            }
        }
    }
}

void initializeHexapod() {
    try {
        // Crear el controlador principal del estado
        hexapodController = new StateController();

        // Inicializar el controlador
        hexapodController->init();

        // Configurar el modelo si las posiciones de las articulaciones están inicializadas
        if (hexapodController->jointPositionsInitialised()) {
            hexapodController->initModel(false);
        } else {
            // Usar posiciones por defecto si no hay datos de sensores
            hexapodController->initModel(true);
        }

        Serial.println("StateController inicializado");

    } catch (const std::exception &e) {
        Serial.printf("Error inicializando hexápodo: %s\n", e.what());
    }
}

void processMovementCommands() {
    if (!hexapodController)
        return;

    // Crear mensaje de velocidad
    geometry_msgs::Twist velocityMsg;
    velocityMsg.linear.x = linearVelocityX;
    velocityMsg.linear.y = linearVelocityY;
    velocityMsg.linear.z = 0.0;
    velocityMsg.angular.x = 0.0;
    velocityMsg.angular.y = 0.0;
    velocityMsg.angular.z = angularVelocity;

    // Enviar comando de velocidad al controlador
    hexapodController->bodyVelocityInputCallback(velocityMsg);
}

void executeWalkingGait() {
    if (!hexapodController)
        return;

    // Verificar si el robot está en estado de funcionamiento
    SystemState currentState = hexapodController->getSystemState();

    if (currentState == SUSPENDED) {
        // Transición a estado operacional
        hexapodController->systemStateCallback(OPERATIONAL);
    }

    // Configurar marcha trípode como predeterminada
    hexapodController->gaitSelectionCallback(TRIPOD_GAIT);
}

void handlePoseControl() {
    if (!hexapodController)
        return;

    // Crear mensaje de pose
    geometry_msgs::Twist poseMsg;
    poseMsg.linear.x = 0.0;        // Translación X
    poseMsg.linear.y = 0.0;        // Translación Y
    poseMsg.linear.z = bodyHeight; // Altura del cuerpo
    poseMsg.angular.x = bodyRoll;  // Rotación Roll
    poseMsg.angular.y = bodyPitch; // Rotación Pitch
    poseMsg.angular.z = bodyYaw;   // Rotación Yaw

    // Enviar comando de pose al controlador
    hexapodController->bodyPoseInputCallback(poseMsg);
}

void updateLegStates() {
    if (!hexapodController)
        return;

    // Publicar estados deseados de las articulaciones
    hexapodController->publishDesiredJointState();

    // Publicar información de depuración si está habilitada
    hexapodController->publishLegState();
    hexapodController->publishVelocity();
    hexapodController->publishPose();
}

// Funciones auxiliares para demostración de patrones de marcha específicos
void demonstrateTripodGait() {
    Serial.println("Demostrando marcha trípode...");

    // Configurar marcha trípode
    hexapodController->gaitSelectionCallback(TRIPOD_GAIT);

    // Movimiento hacia adelante
    linearVelocityX = 0.3;
    linearVelocityY = 0.0;
    angularVelocity = 0.0;

    delay(3000); // 3 segundos de movimiento

    // Detener
    linearVelocityX = 0.0;
    linearVelocityY = 0.0;
    angularVelocity = 0.0;
}

void demonstrateRippleGait() {
    Serial.println("Demostrando marcha ondulante...");

    // Configurar marcha ondulante
    hexapodController->gaitSelectionCallback(RIPPLE_GAIT);

    // Movimiento hacia adelante más lento
    linearVelocityX = 0.2;
    linearVelocityY = 0.0;
    angularVelocity = 0.0;

    delay(4000); // 4 segundos de movimiento

    // Detener
    linearVelocityX = 0.0;
    linearVelocityY = 0.0;
    angularVelocity = 0.0;
}

void demonstratePoseControl() {
    Serial.println("Demostrando control de pose...");

    // Secuencia de poses
    float poses[][4] = {
        {0.0, 0.1, 0.0, 0.15}, // Pitch hacia adelante
        {0.1, 0.0, 0.0, 0.15}, // Roll a la derecha
        {0.0, 0.0, 0.2, 0.15}, // Yaw rotación
        {0.0, 0.0, 0.0, 0.20}, // Altura aumentada
        {0.0, 0.0, 0.0, 0.10}, // Altura reducida
        {0.0, 0.0, 0.0, 0.15}  // Pose neutral
    };

    for (int i = 0; i < 6; i++) {
        bodyRoll = poses[i][0];
        bodyPitch = poses[i][1];
        bodyYaw = poses[i][2];
        bodyHeight = poses[i][3];

        Serial.printf("Pose %d: roll=%.2f, pitch=%.2f, yaw=%.2f, height=%.2f\n",
                      i + 1, bodyRoll, bodyPitch, bodyYaw, bodyHeight);

        delay(2000); // Mantener pose por 2 segundos
    }
}