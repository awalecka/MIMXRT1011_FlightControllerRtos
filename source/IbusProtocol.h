#ifndef IBUS_PROTOCOL_H
#define IBUS_PROTOCOL_H

#include <vector>   // For dynamic arrays (e.g., std::vector<unsigned char>)
#include <numeric>  // For std::accumulate (though custom loop is used for checksum)

// Define states for the state machine
enum State {
    STATE_IDLE = 0,
    STATE_WAIT_HEADER_2,
    STATE_RECEIVE_CHANNEL_DATA,
    STATE_RECEIVE_CHECKSUM_LOW,
    STATE_RECEIVE_CHECKSUM_HIGH
};

class IbusProtocol {
public:
    // Protocol constants
    static const unsigned char HEADER_BYTE_1 = 0x20;
    static const unsigned char HEADER_BYTE_2 = 0x40;
    static const int NUM_CHANNEL_PAIRS = 14;
    static const int BYTES_PER_CHANNEL = 2;
    static const int TOTAL_CHANNEL_BYTES = NUM_CHANNEL_PAIRS * BYTES_PER_CHANNEL;
    static const int MESSAGE_LENGTH = 2 + TOTAL_CHANNEL_BYTES + 2; // Header (2) + Channels (28) + Checksum (2) = 32 bytes
    static const int MAX_MESSAGE_SIZE = MESSAGE_LENGTH;

    /**
     * @brief Constructor for IbusProtocol.
     */
    IbusProtocol();

    /**
     * @brief Processes a single incoming byte and updates the state machine.
     */
    void processByte(unsigned char byte_value);

    const std::vector<unsigned short>& getChannels() const;
    int getParsedMessageCount() const;
    int getErrorCount() const;
    bool isLastMessageValid() const;

private:
    State currentState;
    std::vector<unsigned char> receivedBytes;
    unsigned short expectedChecksum;
    std::vector<unsigned short> channels;
    unsigned short actualChecksum;
    int parsedMessageCount;
    int errorCount;
    bool lastMessageValid;

    void resetMessage(bool resetValid);
    void updateChecksum(unsigned char byte_value);
};

#endif // IBUS_PROTOCOL_H
