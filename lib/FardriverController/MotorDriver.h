#pragma once

#include <inttypes.h>
#include <string.h>

// swaps uint16 endian
inline void swap_endian(uint16_t &val)
{
    val = (val << 8) | (val >> 8);
}

struct motor_data_t
{
    uint32_t time; // время последнего обновления данных. TODO: а оно нам надо?
    uint8_t voltage;
    int16_t current;
    int16_t temperature;
    // и т.п.
};

using time_function_t = uint32_t (*)();

/*********************************************************************************************************
 *********************************************************************************************************/
class MotorManagerInterface
{
public:
    virtual void RXEvent(uint8_t idx, uint8_t *buffer, uint8_t length) = 0;
    virtual void TXEvent(uint8_t idx, uint8_t *buffer, uint8_t length) = 0;
    virtual void Processing(uint32_t time) = 0;
};

/*********************************************************************************************************
 *********************************************************************************************************/
class MotorParserInterface
{
public:
    enum parser_state_code_t : uint8_t
    {
        STATE_ERROR = 0x00,
        STATE_DISCONNECTED = 0x01,
        STATE_PAIRING = 0x02,
        STATE_CONNECTED_PARSING = 0x03,
        STATE_CONNECTED_PERIODIC = 0x04,

        STATE_IGNORE = 0xFF, // используем, если не хотим менять базовое поведение машины состояний
    };

    struct parser_state_t
    {
        uint32_t enter_time;
        parser_state_code_t state_code;
    };

    MotorParserInterface() = delete;

    MotorParserInterface(time_function_t time_func, MotorManagerInterface &manager, motor_data_t &motor_data)
    {
        _current_state.state_code = STATE_DISCONNECTED;
        _current_state.enter_time = 0;

        SetTimeFunction(time_func);
        SetManager(manager);
        SetMotorData(motor_data);
    }

    virtual ~MotorParserInterface() = default;

    void SetTimeFunction(time_function_t time_func)
    {
        _time_func = time_func;
    }

    void SetManager(MotorManagerInterface &manager)
    {
        _manager = &manager;
    }

    void SetMotorData(motor_data_t &motor_data)
    {
        _motor_data = &motor_data;
    }

    // Проверка данных на валидность (AT+ команда или пакет, 0xAA, старший бит второго байта, CRC)
    bool IsValidHandshakeOrPacket(uint8_t *buffer, uint8_t length)
    {
        return IsValidHandshake(buffer, length) || IsValidPacket(buffer, length);
    };

    bool IsValidHandshake(uint8_t *buffer, uint8_t length)
    {
        if (buffer == nullptr)
            return false;

        return _HasValidHandshake(buffer, length);
    };

    bool IsValidPacket(uint8_t *buffer, uint8_t length)
    {
        if (buffer == nullptr)
            return false;

        return _HasValidPacket(buffer, length);
    };

    /// @brief ищет первое вхождение валидного пакета или хендшейка в буфере
    /// @param buffer буфер для поиска
    /// @param length длина данных ф буфере
    /// @param offset [OUT] переменная, в которой в случае успешности поиска будет храниться смещение начала пакета в буфере
    /// @return 'true' в случае, если пакет найден
    ///         'false', если пакет в буфере отсутствует
    bool FindValidPacketOffset(uint8_t *buffer, uint8_t length, uint8_t &offset)
    {
        offset = 0;
        for (uint8_t i = 0; i < length; i++)
        {
            if (IsValidHandshakeOrPacket(&buffer[i], length - i))
            {
                offset = i;
                return true;
            }
        }
        return false;
    }

    parser_state_t GetCurrentState() { return _current_state; };
    parser_state_code_t GetCurrentStateCode() { return _current_state.state_code; };
    uint32_t GetCurrentStateTime() { return _current_state.enter_time; };

    /// @brief Выполняет всю основную работу
    /// @param time Время вызова в мсек
    /// @param motor_data Структура распарсенных данных по мотору. Заполняется, если есть пакет.
    /// @param buffer Указатель на буфер с сырыми данными из UART
    /// @param length Длина данных в буфере
    /// @return Возвращает число обработанных байт в буфере (менеджер сможет их удалить).
    uint8_t Process(uint8_t *buffer = nullptr, uint8_t length = 0)
    {
        uint8_t bytes_processed = 0;

        switch (_current_state.state_code)
        {
        case STATE_DISCONNECTED:
        {
            _SetState(_OnStateDisconnectedBaseHandler(buffer, length));
            break;
        };
        case STATE_PAIRING:
        {
            _SetState(_OnStatePairingBaseHandler(buffer, length));
            break;
        };

        case STATE_CONNECTED_PARSING:
        {
            if (IsValidPacket(buffer, length))
            {
                // стейт не меняем, обрабатываем пакет
                bytes_processed = _ParsePacket(buffer, length);
                // увеличиваем счетчик пакетов, если их надо считать
                if (_parsing_to_periodic_packet_limit > 0 && _processed_packets_count < UINT8_MAX)
                    _processed_packets_count++;
            }
            _SetState(_OnStateConnectedParsingBaseHandler(buffer, length));
            break;
        };

        case STATE_CONNECTED_PERIODIC:
        {
            _SetState(_OnStateConnectedPeriodicBaseHandler(buffer, length));
            break;
        };

        case STATE_IGNORE:
        {
            break;
        }

        case STATE_ERROR:
        default:
        {
            _SetState(_OnStateErrorBaseHandler(buffer, length));
            break;
        };
        }

        // если есть, что отправить, отправляем
        if (_tx_buffer_data_size > 0)
        {
            _manager->TXEvent(_motor_index, _tx_buffer, _tx_buffer_data_size);
            memset(_tx_buffer, 0, _tx_buffer_data_size);
            _tx_buffer_data_size = 0;
        }

        return bytes_processed;
    };

protected:
    static constexpr uint8_t _tx_buffer_max_length = 32;
    uint8_t _tx_buffer[_tx_buffer_max_length] = {0};
    uint8_t _tx_buffer_data_size = 0;

    uint8_t _motor_index = 0;

    // параметры ниже сделаны не константами, чтобы в парсерах-потомках можно было их менять
    uint8_t _disconnected_to_error_limit = 30;        // после этого количества попадений в состояние дисконнекта перейдем в состояние ошибки; если 0, то отключен
    uint32_t _pairing_to_disconnected_timeout = 3000; // ms, таймаут ожидания пакетов после ответа на хендшейк
    uint32_t _parsing_to_periodic_timeout = 550;      // ms, таймаут перехода из Connected.Parsing в Connected.Periodic; если 0, то отключен
    uint8_t _parsing_to_periodic_packet_limit = 0;    // число пакетов, после которого выполняется переход из Connected.Parsing в Connected.Periodic; если 0, то отключен

    parser_state_t _current_state;
    uint8_t _state_disconnected_reached_count = 0; // сколько раз свалились в состояние дисконнекта
    uint8_t _processed_packets_count = 0;          // сколько пакетов обработали

    time_function_t _time_func = nullptr;
    MotorManagerInterface *_manager = nullptr;
    motor_data_t *_motor_data = nullptr;

    void _SetState(parser_state_code_t state_code)
    {
        // обработка только при действительной смене состояния
        if (_current_state.state_code == state_code || state_code == STATE_IGNORE)
            return;

        switch (state_code)
        {
        case STATE_DISCONNECTED:
        {
            // увеличиваем счетчик обрывов коннекта
            if (_disconnected_to_error_limit != 0 && _state_disconnected_reached_count < UINT8_MAX)
                _state_disconnected_reached_count++;

            break;
        }

        case STATE_ERROR:
        {
            _state_disconnected_reached_count = 0;
            break;
        }

        case STATE_PAIRING:
        {
            _tx_buffer_data_size += _FillHandshakeResponse(_tx_buffer, _tx_buffer_max_length - _tx_buffer_data_size);
            break;
        }

        case STATE_CONNECTED_PARSING:
        {
            break;
        }

        case STATE_CONNECTED_PERIODIC:
        {
            _tx_buffer_data_size += _FillPacketRequest(_tx_buffer, _tx_buffer_max_length - _tx_buffer_data_size);
            _processed_packets_count = 0;
            break;
        }

        case STATE_IGNORE:
        default:
            break;
        }

        _current_state.enter_time = _time_func();
        _current_state.state_code = state_code;
    }

    parser_state_code_t _OnStateDisconnectedBaseHandler(uint8_t *buffer, uint8_t length)
    {
        parser_state_code_t result = _OnStateDisconnected(buffer, length);
        if (result != STATE_IGNORE)
            return result;

        if (IsValidHandshake(buffer, length))
        {
            result = STATE_PAIRING;
        }
        else if (IsValidPacket(buffer, length))
        {
            result = STATE_CONNECTED_PARSING;
        }
        else if (_state_disconnected_reached_count >= _disconnected_to_error_limit)
        {
            result = STATE_ERROR;
        }

        return result;
    }

    // виртуальный обработчик для переопределения в классах-потомках
    virtual parser_state_code_t _OnStateDisconnected(uint8_t *buffer, uint8_t length) { return STATE_IGNORE; };

    parser_state_code_t _OnStatePairingBaseHandler(uint8_t *buffer, uint8_t length)
    {
        parser_state_code_t result = _OnStatePairing(buffer, length);
        if (result != STATE_IGNORE)
            return result;

        if (IsValidPacket(buffer, length))
        {
            result = STATE_CONNECTED_PARSING;
        }
        else if (buffer != nullptr && length > 0 && !IsValidHandshakeOrPacket(buffer, length))
        { // TODO: возможно, проверка тут на инвалидность данных будет ломать процесс, если буфер стартует с обрубка пакета
            result = STATE_DISCONNECTED;
        }
        else if (_time_func() - _current_state.enter_time >= _pairing_to_disconnected_timeout)
        {
            result = STATE_DISCONNECTED;
        }

        return result;
    }

    // виртуальный обработчик для переопределения в классах-потомках
    virtual parser_state_code_t _OnStatePairing(uint8_t *buffer, uint8_t length) { return STATE_IGNORE; };

    parser_state_code_t _OnStateConnectedParsingBaseHandler(uint8_t *buffer, uint8_t length)
    {
        parser_state_code_t result = _OnStateConnectedParsing(buffer, length);
        if (result != STATE_IGNORE)
            return result;

        if (IsValidPacket(buffer, length))
        {
            // обработчик в методе Process
        }
        else if (IsValidHandshake(buffer, length))
        {
            result = STATE_DISCONNECTED;
        }
        else if (buffer != nullptr && length > 0 && !IsValidHandshakeOrPacket(buffer, length))
        {
            result = STATE_DISCONNECTED;
        }

        if (_parsing_to_periodic_packet_limit > 0 && _processed_packets_count >= _parsing_to_periodic_packet_limit)
        {
            result = STATE_CONNECTED_PERIODIC;
        }
        else if (_parsing_to_periodic_timeout > 0 && (_time_func() - _current_state.enter_time >= _parsing_to_periodic_timeout))
        {
            result = STATE_CONNECTED_PERIODIC;
        }

        return result;
    }

    // виртуальный обработчик для переопределения в классах-потомках
    virtual parser_state_code_t _OnStateConnectedParsing(uint8_t *buffer, uint8_t length) { return STATE_IGNORE; };

    parser_state_code_t _OnStateConnectedPeriodicBaseHandler(uint8_t *buffer, uint8_t length)
    {
        parser_state_code_t result = _OnStateConnectedPeriodic(buffer, length);
        if (result != STATE_IGNORE)
            return result;

        return result;
    }

    // виртуальный обработчик для переопределения в классах-потомках
    virtual parser_state_code_t _OnStateConnectedPeriodic(uint8_t *buffer, uint8_t length) { return STATE_IGNORE; };

    parser_state_code_t _OnStateErrorBaseHandler(uint8_t *buffer, uint8_t length)
    {
        // TODO: сделать тут информирование об ошибке, если надо

        parser_state_code_t result = _OnStateError(buffer, length);
        if (result != STATE_IGNORE)
            return result;

        // по дефолту из состояния ошибки должны выпадать в состояние "нет коннекта"
        return STATE_DISCONNECTED;
    }

    // виртуальный обработчик для переопределения в классах-потомках
    virtual parser_state_code_t _OnStateError(uint8_t *buffer, uint8_t length) { return STATE_IGNORE; };

    // проверяет наличие правильного хендшейка в буфере (смотрит с 1 позиции буфера, весь буфер не просматривает)
    virtual bool _HasValidHandshake(uint8_t *buffer, uint8_t length) = 0;

    // возвращает длину записанного в буфер ответа на хендшейк
    // если 0, значит ответа нет (отвечать не надо)
    virtual uint8_t _FillHandshakeResponse(uint8_t *response_buffer, uint8_t response_buffer_max_length) = 0;

    // проверяет наличие правильной стартовой последовательности пакета в начале буфера
    virtual bool _HasValidPacketStart(uint8_t *buffer, uint8_t length) = 0;

    // проверяет наличие корректного пакета в буфере (стартовый байт, контрольная сумма, длина и т.п.)
    virtual bool _HasValidPacket(uint8_t *buffer, uint8_t length) = 0;

    // возвращает длину записанного в буфер запроса пакетов с данными
    // если 0, значит запроса нет (ничего запрашивать не надо)
    virtual uint8_t _FillPacketRequest(uint8_t *request_buffer, uint8_t request_buffer_max_length) = 0;

    /// @brief Разбирает пакет с начала буфера, заполняя данные в структуру мотора
    /// @param buffer Буфер с пакетом
    /// @param length Длина данных в буфере
    /// @return число байт — длина обработанного пакета, даже если из него никакие данные извлечены не были (но он был валидный).
    ///         '0' — если обработка не удалась (пакет не валидный или в буфере не пакет вовсе).
    virtual uint8_t _ParsePacket(uint8_t *buffer, uint8_t length) = 0;
};

/*********************************************************************************************************
 *********************************************************************************************************/
/*********************************************************************************************************
 *********************************************************************************************************/
class FardriverParser2022 : public MotorParserInterface
{
public:
    FardriverParser2022(time_function_t time_func, MotorManagerInterface &manager, motor_data_t &motor_data) : MotorParserInterface(time_func, manager, motor_data){};

private:
    static constexpr uint8_t _handshake_start_bytes_count = 3;
    static constexpr uint8_t _handshake_request[] = {'A', 'T', '+', 'P', 'A', 'S', 'S', '=', '2', '9', '6', '8', '8', '7', '8', '1'};

    static constexpr uint8_t _handshake_response[] = {'+', 'P', 'A', 'S', 'S', '=', 'O', 'N', 'N', 'D', 'O', 'N', 'K', 'E'};

    static constexpr uint8_t _packet_start_byte = 0xAA;
    static constexpr uint8_t _packet_length = 16;

    static constexpr uint8_t _packet_request[] = {0xAA, 0x13, 0xEC, 0x07, 0x09, 0x6F, 0x28, 0xD7};

protected:
    virtual bool _HasValidHandshake(uint8_t *buffer, uint8_t length) override
    {
        if (buffer == nullptr || length < sizeof(_handshake_request))
            return false;

        return (memcmp(buffer, _handshake_request, sizeof(_handshake_request)) == 0);
    };

    // возвращает длину записанного ответа на хендшейк
    virtual uint8_t _FillHandshakeResponse(uint8_t *response_buffer, uint8_t response_buffer_max_length) override
    {
        if (response_buffer == nullptr || response_buffer_max_length < sizeof(_handshake_response))
            return 0;

        memcpy(response_buffer, _handshake_response, sizeof(_handshake_response));

        return sizeof(_handshake_response);
    };

    virtual bool _HasValidPacketStart(uint8_t *buffer, uint8_t length) override
    {
        if (buffer == nullptr || length < 2)
            return false;

        return ((buffer[0] == _packet_start_byte) && ((buffer[1] & 0x80) == 0));
    };

    virtual bool _HasValidPacket(uint8_t *buffer, uint8_t length) override
    {
        if (buffer == nullptr)
            return false;

        return (length >= _packet_length) &&
               _HasValidPacketStart(buffer, length) &&
               _IsCRCCorrect(buffer, length);
    };

    // возвращает длину записанного в буфер запроса пакетов с данными
    // если 0, значит запроса нет (ничего запрашивать не надо)
    uint8_t _FillPacketRequest(uint8_t *request_buffer, uint8_t request_buffer_max_length) override
    {
        if (request_buffer == nullptr || request_buffer_max_length < sizeof(_packet_request))
            return 0;

        memcpy(request_buffer, _packet_request, sizeof(_packet_request));

        return sizeof(_packet_request);
    };

    /// @brief Разбирает пакет с начала буфера, заполняя данные в структуру мотора
    /// @param buffer Буфер с пакетом
    /// @param length Длина данных в буфере
    /// @return число байт — длина обработанного пакета, даже если из него никакие данные извлечены не были (но он был валидный).
    ///         '0' — если обработка не удалась (пакет не валидный или в буфере не пакет вовсе).
    virtual uint8_t _ParsePacket(uint8_t *buffer, uint8_t length) override
    {
        return 0;
    };

    // В этой версии контрольная сумма - просто алгебраическая сумма байт
    // TODO: не знаю, входит ли стартовый байт в сумму, надо уточнить
    bool _IsCRCCorrect(uint8_t *buffer, uint8_t length)
    {
        if (buffer == nullptr || length < _packet_length)
            return false;

        uint16_t crc = 0x0000;

        // суммируем все байты пакета, кроме CRC
        for (uint8_t i = 0; i < _packet_length - 2; ++i)
        {
            crc += buffer[i];
        }

        // теперь указатель смотрит на 2 последних байта в пакете (CRC)
        buffer += _packet_length - 2;

        swap_endian(*(uint16_t *)buffer);

        return crc == *(uint16_t *)buffer;
    };
};

/*********************************************************************************************************
 *********************************************************************************************************/
class FardriverParser2023 : public MotorParserInterface
{
public:
    FardriverParser2023(time_function_t time_func, MotorManagerInterface &manager, motor_data_t &motor_data) : MotorParserInterface(time_func, manager, motor_data){};

private:
    static constexpr uint8_t _handshake_start_bytes_count = 3;
    static constexpr uint8_t _handshake_request[] = {'A', 'T', '+', 'V', 'E', 'R', 'S', 'I', 'O', 'N'};

    static constexpr uint8_t _handshake_response[] = {0xAA, 0x13, 0xEC, 0x07, 0x01, 0xF1, 0xA2, 0x5D};

    static constexpr uint8_t _packet_start_byte = 0xAA;
    static constexpr uint8_t _packet_length = 16;

protected:
    virtual bool _HasValidHandshake(uint8_t *buffer, uint8_t length) override
    {
        if (buffer == nullptr || length < sizeof(_handshake_request))
            return false;

        return (memcmp(buffer, _handshake_request, sizeof(_handshake_request)) == 0);
    };

    // возвращает длину записанного ответа на хендшейк
    virtual uint8_t _FillHandshakeResponse(uint8_t *response_buffer, uint8_t response_buffer_max_length) override
    {
        if (response_buffer == nullptr || response_buffer_max_length < sizeof(_handshake_response))
            return 0;

        memcpy(response_buffer, _handshake_response, sizeof(_handshake_response));

        return sizeof(_handshake_response);
    };

    virtual bool _HasValidPacketStart(uint8_t *buffer, uint8_t length) override
    {
        if (buffer == nullptr || length < 2)
            return false;

        return ((buffer[0] == _packet_start_byte) && ((buffer[1] & 0x80) == 0x80));
    };

    virtual bool _HasValidPacket(uint8_t *buffer, uint8_t length) override
    {
        if (buffer == nullptr)
            return false;

        return (length >= _packet_length) &&
               _HasValidPacketStart(buffer, length) &&
               _IsCRCCorrect(buffer, length);
    };

    // возвращает длину записанного в буфер запроса пакетов с данными
    // если 0, значит запроса нет (ничего запрашивать не надо)
    uint8_t _FillPacketRequest(uint8_t *request_buffer, uint8_t request_buffer_max_length) override
    {
        return 0;
    };

    /// @brief Разбирает пакет с начала буфера, заполняя данные в структуру мотора
    /// @param buffer Буфер с пакетом
    /// @param length Длина данных в буфере
    /// @return число байт — длина обработанного пакета, даже если из него никакие данные извлечены не были (но он был валидный).
    ///         '0' — если обработка не удалась (пакет не валидный или в буфере не пакет вовсе).
    virtual uint8_t _ParsePacket(uint8_t *buffer, uint8_t length) override
    {
        return 0;
    };

    // В этой версии контрольная сумма - это CRC16 (Poly: 0x8005, Init: 0x7F3C, RefIn: true, RefOut: true, XorOut: false)
    // TODO: не знаю, входит ли стартовый байт в сумму, надо уточнить
    bool _IsCRCCorrect(uint8_t *buffer, uint8_t length)
    {
        if (buffer == nullptr || length < _packet_length)
            return false;

        uint16_t crc = 0x7F3C;

        // считаем все байты пакета, кроме CRC
        for (uint8_t idx = 0; idx < _packet_length - 2; ++idx)
        {
            crc ^= (uint16_t)buffer[idx];
            for (uint8_t i = 8; i != 0; --i)
            {
                if ((crc & 0x0001) != 0)
                {
                    crc >>= 1;
                    crc ^= 0xA001;
                }
                else
                {
                    crc >>= 1;
                }
            }
        }

        // теперь указатель смотрит на 2 последних байта в пакете (CRC)
        buffer += _packet_length - 2;

        return crc == *(uint16_t *)buffer;
    };
};

/*********************************************************************************************************
 *********************************************************************************************************/
class MotorManager : public MotorManagerInterface
{
    static constexpr uint8_t _max_motors = 2;
    static constexpr uint8_t _parsers_count = 2;
    using tx_callback_t = void (*)(const uint8_t idx, const uint8_t *buffer, uint8_t const length);

public:
    MotorManager(tx_callback_t tx_callback)
    {
        _tx_callback = tx_callback;
        for (uint8_t i = 0; i < _max_motors; i++)
        {
            // init motors array
        }

        for (uint8_t i = 0; i < _parsers_count; i++)
        {
            _parsers[i] = nullptr;
        }

        _parsers[0] = new FardriverParser2022(millis, *this, _motors[0].motor_data);
        _parsers[1] = new FardriverParser2023(millis, *this, _motors[1].motor_data);
    }

    ~MotorManager()
    {
        for (uint8_t i = 0; i < _parsers_count; i++)
        {
            if (_parsers[i] != nullptr)
                delete _parsers[i];
        }
    };

    virtual void RXEvent(uint8_t idx, uint8_t *buffer, uint8_t length) override
    {
        if (idx >= _max_motors)
            return;

        memcpy(_motors[idx].rx_buffer, buffer, length);
        _motors[idx].rx_data_size = length;
    }

    virtual void TXEvent(uint8_t idx, uint8_t *buffer, uint8_t length) override
    {
        if (_tx_callback != nullptr)
        {
            _tx_callback(idx, buffer, length);
        }
    }

    virtual void Processing(uint32_t time) override
    {
        for (motor_t &motor : _motors)
        {
            // если данные получены...
            if (motor.rx_data_size > 0)
            {
                // если при этом драйвер не выбран или он в состоянии дисконнекта...
                if (motor.parser == nullptr)
                {
                    // ...значит надо выбирать драйвер
                    _SelectDriver(motor);

                    // если драйвер так и не выбран, значит ни одному из драйверов данные не подошли
                    // следовательно данные надо забыть
                    // будем ждать, пока в первой позиции принятой пачки байт не будет валидный хендшейк или пакет
                    if (motor.parser == nullptr)
                    {
                        motor.rx_data_size = 0;
                    }
                }

                // если драйвер выбран...
                if (motor.parser != nullptr)
                {
                    // ...значит надо данные обработать
                }
            }
        }
    }

private:
    typedef struct
    {
        uint8_t motor_idx;     // индекс мотора
        uint8_t rx_buffer[32]; // Приёмный буфер
        uint8_t rx_data_size;  // Длина данных в буфере

        MotorParserInterface *parser;

        motor_data_t motor_data; // распарсенные данные по мотору, только те, что нам нужны
    } motor_t;

    motor_t _motors[_max_motors];

    MotorParserInterface *_parsers[_parsers_count];

    tx_callback_t _tx_callback = nullptr;

    // определяемся, какой драйвер должен обрабатывать данные с мотора
    void _SelectDriver(motor_t &motor)
    {
        /*
        // если драйвер уже выбран и у него всё хорошо со статусом, то нам тут делать нечего
        if (motor.parser != nullptr && motor.parser_state == MotorParserInterface::STATE_CONNECTED)
            // хотя... мы же ничего не поломаем, если при выбранном драйвере выберем его повторно...
            return;

        // пробуем старый драйвер
        if (_parser_2022.IsValidPacket(motor.rx_buffer, motor.rx_data_size))
        {
            // если подошел, то сохраняем указатель на него
            motor.parser = (MotorParserInterface *)&_parser_2022;
            return; // подошел
        }

        // старый драйвер не подходит, пробуем новый
        if (_parser_2023.IsValidPacket(motor.rx_buffer, motor.rx_data_size))
        {
            motor.parser = (MotorParserInterface *)&_parser_2023;
            return; // подошел
        }
        */

        // новый тоже не подошел, печаль
        return;
    }
};
