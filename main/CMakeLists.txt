
file(GLOB AWS_SRCS aws/*.cpp)
file(GLOB UTIL_SRCS util/*.cpp)
file(GLOB INTERFACE_SRCS interface/*.cpp)
file(GLOB BME280_SRCS bme280/*.cpp)
file(GLOB COREMQTT_SRCS aws-iot-device-sdk-embedded-C/libraries/standard/coreMQTT/source/*.c)
file(GLOB BACKOFF_ALG_SRCS aws-iot-device-sdk-embedded-C/libraries/standard/backoffAlgorithm/source/*.c)

set(COMPONENT_SRCS ${AWS_SRCS} ${UTIL_SRCS} ${INTERFACE_SRCS} ${BME280_SRCS} ${COREMQTT_SRCS} ${BACKOFF_ALG_SRCS} "sim7080_demo.cpp")
set(COMPONENT_ADD_INCLUDEDIRS 
    "." 
    "setting/"
    "aws-iot-device-sdk-embedded-C/libraries/standard/coreMQTT/source/include"
    "aws-iot-device-sdk-embedded-C/libraries/standard/coreMQTT/source/interface"
    "aws-iot-device-sdk-embedded-C/libraries/standard/backoffAlgorithm/source/include")

register_component()

target_add_binary_data(${COMPONENT_TARGET} "certs/aws-root-ca.pem" TEXT)
target_add_binary_data(${COMPONENT_TARGET} "certs/certificate.pem.crt" TEXT)
target_add_binary_data(${COMPONENT_TARGET} "certs/private.pem.key" TEXT)
