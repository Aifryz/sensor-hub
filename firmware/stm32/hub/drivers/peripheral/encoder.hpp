#include <cstdint>
class encoder
{
    public:

    enum class InputState: uint8_t
    {
        A0B0 = 0b00,
        A0B1 = 0b01,
        A1B0 = 0b10,
        A1B1 = 0b11,
        None = 0xFF
    };

    int32_t get_ticks()
    {
        return m_ticks/4;
    }

    int32_t get_full_ticks()
    {
        return m_ticks;
    }

    void update_state(InputState state)
    {
        if(state == m_last_state)
            return;

        if(state == InputState::None)
        {
            m_last_state = state;
            return;
        }
        else
        {
            // State transition table for quadrature encoder
            // Each state is represented as a 2-bit value (A and B)
            // The table indicates the change in ticks for each transition
            static const int8_t transition_table[4][4] = {
                { 0, -1,  1,  0}, // From A0B0
                { 1,  0,  0, -1}, // From A0B1
                {-1,  0,  0,  1}, // From A1B0
                { 0,  1, -1,  0}  // From A1B1
            };

            uint8_t last_state_index = static_cast<uint8_t>(m_last_state);
            uint8_t new_state_index = static_cast<uint8_t>(state);

            m_ticks += transition_table[last_state_index][new_state_index];
            m_last_state = state;
        }
    }

    private:
    InputState m_last_state = InputState::None;
    int32_t m_ticks;
};

extern encoder enc;