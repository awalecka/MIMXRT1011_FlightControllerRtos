#include "IbusProtocol.h"

/**
 * @brief Constructs the IbusProtocol object and resets internal state.
 */
IbusProtocol::IbusProtocol() {
    resetMessage(true);
}

/**
 * @brief Processes a single byte in the IBUS protocol stream.
 * @param byte_value The byte to process.
 */
void IbusProtocol::processByte(unsigned char byte_value) {
    if (receivedBytes.size() >= MAX_MESSAGE_SIZE) {
        errorCount++;
        resetMessage(true);
        return;
    }

    receivedBytes.push_back(byte_value);

    switch (currentState) {
        case STATE_IDLE:
            if (byte_value == HEADER_BYTE_1) {
                currentState = STATE_WAIT_HEADER_2;
                updateChecksum(byte_value);
            } else {
                resetMessage(true);
            }
            break;

        case STATE_WAIT_HEADER_2:
            if (byte_value == HEADER_BYTE_2) {
                currentState = STATE_RECEIVE_CHANNEL_DATA;
                updateChecksum(byte_value);
            } else {
                errorCount++;
                resetMessage(true);
            }
            break;

        case STATE_RECEIVE_CHANNEL_DATA:
            if (receivedBytes.size() < (2 + TOTAL_CHANNEL_BYTES)) {
                updateChecksum(byte_value);
            } else if (receivedBytes.size() == (2 + TOTAL_CHANNEL_BYTES)) {
                updateChecksum(byte_value);

                for (int i = 0; i < NUM_CHANNEL_PAIRS; ++i) {
                    int low_byte_idx = 2 + (i * BYTES_PER_CHANNEL);
                    int high_byte_idx = low_byte_idx + 1;

                    if (high_byte_idx < static_cast<int>(receivedBytes.size())) {
                        unsigned char low_byte = receivedBytes[low_byte_idx];
                        unsigned char high_byte = receivedBytes[high_byte_idx];
                        channels[i] = static_cast<unsigned short>(low_byte | (high_byte << 8));
                    } else {
                        errorCount++;
                        resetMessage(true);
                        return;
                    }
                }
                currentState = STATE_RECEIVE_CHECKSUM_LOW;
            }
            break;

        case STATE_RECEIVE_CHECKSUM_LOW:
            actualChecksum = byte_value;
            currentState = STATE_RECEIVE_CHECKSUM_HIGH;
            break;

        case STATE_RECEIVE_CHECKSUM_HIGH:
            actualChecksum |= static_cast<unsigned short>(byte_value << 8);
            lastMessageValid = ((expectedChecksum & 0xFFFF) == actualChecksum);

            if (!lastMessageValid) {
                errorCount++;
            }

            parsedMessageCount++;
            resetMessage(false);  // Do not reset lastMessageValid here
            break;

        default:
            errorCount++;
            resetMessage(true);
            break;
    }
}

/**
 * @brief Gets the decoded channel values.
 * @return Reference to a vector of channel values.
 */
const std::vector<unsigned short>& IbusProtocol::getChannels() const {
    return channels;
}

/**
 * @brief Gets the number of successfully parsed messages.
 * @return The message count.
 */
int IbusProtocol::getParsedMessageCount() const {
    return parsedMessageCount;
}

/**
 * @brief Gets the number of detected errors.
 * @return The error count.
 */
int IbusProtocol::getErrorCount() const {
    return errorCount;
}

/**
 * @brief Indicates whether the last parsed message was valid.
 * @return True if valid, false otherwise.
 */
bool IbusProtocol::isLastMessageValid() const {
    return lastMessageValid;
}

/**
 * @brief Resets the internal state to prepare for a new message.
 * @param resetValid Whether to reset the lastMessageValid flag.
 */
void IbusProtocol::resetMessage(bool resetValid) {
    currentState = STATE_IDLE;
    receivedBytes.clear();
    expectedChecksum = 0xFFFF;
    channels.assign(NUM_CHANNEL_PAIRS, 0);
    actualChecksum = 0;
    if (resetValid) {
        lastMessageValid = false;
    }
}

/**
 * @brief Updates the checksum by subtracting the provided byte value.
 * @param byte_value Byte to subtract from checksum.
 */
void IbusProtocol::updateChecksum(unsigned char byte_value) {
    expectedChecksum -= byte_value;
    expectedChecksum &= 0xFFFF;
}
