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
    uint16_t voltage;
    int16_t current;
    int16_t temperature;
    // и т.п.
};

using tx_callback_t = void (*)(const uint8_t motor_idx, const uint8_t *buffer, uint8_t const length);

/*********************************************************************************************************
 *********************************************************************************************************/
class MotorManagerInterface
{
public:
    virtual void RXEvent(uint8_t motor_idx, uint8_t *buffer, uint8_t length) = 0;
    virtual void TXEvent(uint8_t motor_idx, const uint8_t *buffer, uint8_t length) = 0;
    virtual void Processing(uint32_t time) = 0;
    virtual void DeleteNFirstBytesFromBuffer(uint8_t motor_idx, uint8_t number_of_bytes) = 0;
    virtual motor_data_t *GetMotorData(uint8_t motor_idx) = 0;
};

/*********************************************************************************************************
 *********************************************************************************************************/
class MotorParserInterface
{
public:
    virtual ~MotorParserInterface() = default;

    void SetNextParser(MotorParserInterface *parser)
    {
        _next_parser = parser;
    };

    bool HasNextParser()
    {
        return _next_parser != nullptr;
    };

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
        if (buffer == nullptr || length == 0)
            return false;

        return _HasValidPacket(buffer, length);
    };

    /// @brief ищет первое вхождение валидного пакета или хендшейка в буфере
    /// @param buffer буфер для поиска
    /// @param length длина данных в буфере
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

    /// @brief Вызывается, если получен корректный хендшейк
    /// @param manager Переданный по ссылке менеджер моторов
    /// @param motor_idx Индекс мотора, с которым работаем
    /// @param buffer буфер с хендшейком
    /// @param length длина данных в буфере
    virtual void OnHandshakeReceived(MotorManagerInterface &manager, uint8_t motor_idx, uint8_t *buffer, uint8_t length) = 0;

    /// @brief Вызывается, если получен корректный пакет.
    /// @param manager Переданный по ссылке менеджер моторов
    /// @param motor_idx Индекс мотора, с которым работаем
    /// @param buffer буфер с хендшейком
    /// @param length длина данных в буфере
    virtual void OnPacketReceived(MotorManagerInterface &manager, uint8_t motor_idx, uint8_t *buffer, uint8_t length) = 0;

    MotorParserInterface *ParsePacket(MotorManagerInterface &manager, uint8_t motor_idx, motor_data_t &motor_data, uint8_t *buffer, uint8_t length)
    {
        uint8_t number_of_bytes_to_delete = 0;

        // если в начале буфера есть валидный хендшейк или пакет, то:
        //     - разбираем пакет и заполняем структуру с данными по мотору
        //     - информируем менеджера, сколько байт из буфера обработано и их можно удалить
        //     - если нужно, с помощью менеджера отправляем в УАРТ ответы и/или запросы
        if (IsValidHandshakeOrPacket(buffer, length))
        {
            if (IsValidHandshake(buffer, length))
            {
                // отрабатываем, что нужно в этом случае по протоколу
                OnHandshakeReceived(manager, motor_idx, buffer, length);
                // и сообщаем менеджеру, что хендшейк из буфера можно удалять
                manager.DeleteNFirstBytesFromBuffer(motor_idx, _GetHandshakeRequestLength());
            }
            else // packet received
            {
                // отрабатываем, что нужно в этом случае по протоколу
                OnPacketReceived(manager, motor_idx, buffer, length);
                // парсим данные
                number_of_bytes_to_delete = _ParsePacket(motor_data, buffer, length);
                // и сообщаем менеджеру, что распарсенный пакет можно удалять из буфера
                manager.DeleteNFirstBytesFromBuffer(motor_idx, number_of_bytes_to_delete);
            }
            return this;
        }
        // если получается найти валидный хендшейк/пакет не с начала буфера, то
        // все лишние данные в буфере помечаем на удаление
        // пакет/хендшейк распарсим на следующей итерации
        // +1, -1 и ++ потому, что начальную позицию в буфере уже проверили, не надо на неё тратить время
        else if (FindValidPacketOffset(buffer + 1, length - 1, ++number_of_bytes_to_delete))
        {
            manager.DeleteNFirstBytesFromBuffer(motor_idx, number_of_bytes_to_delete);
            return this;
        }
        // если есть последующий парсер, то пытаемся распарсить пакет с его помощью
        else if (HasNextParser())
        {
            return _next_parser->ParsePacket(manager, motor_idx, motor_data, buffer, length);
        }

        // ничего не получилось, уходим, обнуляя указатель на последний использованный парсер.
        // Очищать данные в буфере не надо, так как в нем может просто лежать неполный пакет.
        // Немного странновато, что при неполном пакете текущий парсер у мотора будет сбрасываться
        // и выбор парсера после последующего получения всех данных пакета будет происходить с нуля.
        // Но может и пофиг на это. Затраты не большие, а профита много.
        return nullptr;
    };

    /// @brief Удаляет все последующие парсеры в цепочке
    void DeleteNextParser()
    {
        if (!HasNextParser())
            return;

        _next_parser->DeleteNextParser();
        delete _next_parser;
        _next_parser = nullptr;
    };

protected:
    MotorParserInterface *_next_parser = nullptr;

    // проверяет наличие правильного хендшейка в буфере (смотрит с 1 позиции буфера, весь буфер не просматривает)
    virtual bool _HasValidHandshake(uint8_t *buffer, uint8_t length) = 0;

    // проверяет наличие правильной стартовой последовательности пакета в начале буфера
    virtual bool _HasValidPacketStart(uint8_t *buffer, uint8_t length) = 0;

    // проверяет наличие корректного пакета в буфере (стартовый байт, контрольная сумма, длина и т.п.)
    virtual bool _HasValidPacket(uint8_t *buffer, uint8_t length) = 0;

    /// @brief Возвращает указатель на ожидаемый запрос хендшейка от драйвера
    /// @return Указатель на буфер, содержащий запрос хендшейка, или nullptr, если запроса нет
    virtual const uint8_t *_GetHandshakeRequest()
    {
        return nullptr;
    };

    /// @brief Возвращает длину в байтах ожидаемого от драйвера запроса хендшейка
    /// @return Количество байт в запросе хендшейка или '0', если ответа нет
    virtual uint8_t _GetHandshakeRequestLength()
    {
        return 0;
    };

    /// @brief Возвращает указатель на ответ на хендшейк
    /// @return Указатель на буфер, содержащий ответ на хендшейк, или nullptr, если ответа нет
    virtual const uint8_t *_GetHandshakeResponse()
    {
        return nullptr;
    };

    /// @brief Возвращает длину в байтах ответа на хендшейк
    /// @return Количество байт в ответе на хендшейк или '0', если ответа нет
    virtual uint8_t _GetHandshakeResponseLength()
    {
        return 0;
    };

    /// @brief Возвращает указатель на запрос пакетов с данными
    /// @return Указатель на буфер, содержащий запрос пакетов с данными, или nullptr, если запроса нет
    virtual const uint8_t *_GetPacketRequest()
    {
        return nullptr;
    };

    /// @brief Возвращает длину в байтах запроса пакетов с данными
    /// @return Количество байт в запросе данных или '0', если запроса нет
    virtual uint8_t _GetPacketRequestLength()
    {
        return 0;
    };

    /// @brief Разбирает пакет с начала буфера, заполняя данные в структуру мотора
    /// @param motor_data [OUT] Structure with a parsed motor data
    /// @param buffer Буфер с пакетом
    /// @param length Длина данных в буфере
    /// @return число байт — длина обработанного пакета, даже если из него никакие данные извлечены не были (но он был валидный).
    ///         '0' — если обработка не удалась (пакет не валидный или в буфере не пакет вовсе).
    virtual uint8_t _ParsePacket(motor_data_t &motor_data, uint8_t *buffer, uint8_t length) = 0;
};

/*********************************************************************************************************
 *********************************************************************************************************/
/*********************************************************************************************************
 *********************************************************************************************************/
class FardriverParser2022 : public MotorParserInterface
{
public:
    virtual void OnHandshakeReceived(MotorManagerInterface &manager, uint8_t motor_idx, uint8_t *buffer, uint8_t length) override
    {
        // отправляем ответ на хендшейк
        manager.TXEvent(motor_idx, _handshake_response, sizeof(_handshake_response));
        // сразу же отправляем запрос на данные
        manager.TXEvent(motor_idx, _packet_request, sizeof(_packet_request));
    };

    virtual void OnPacketReceived(MotorManagerInterface &manager, uint8_t motor_idx, uint8_t *buffer, uint8_t length) override
    {
        // Запрос данных в этой версии протокола отправляется примерно каждые 18-19 полученных пакетов.
        // Поэтому делаю запрос после пакета 0x0C и 0x15, то есть запросы будут отправлены после 18 и 20 пакетов соответственно.
        // Не думаю, что это проблема
        if (buffer == nullptr || length == 0)
            return;

        if (buffer[1] == 0x0C || buffer[1] == 0x15)
            manager.TXEvent(motor_idx, _packet_request, sizeof(_packet_request));
    };

private:
    // static constexpr uint8_t _handshake_start_bytes_count = 3;
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

    virtual const uint8_t *_GetHandshakeRequest()
    {
        return _handshake_request;
    };

    virtual uint8_t _GetHandshakeRequestLength()
    {
        return sizeof(_handshake_request);
    };

    virtual const uint8_t *_GetHandshakeResponse() override
    {
        return _handshake_response;
    };

    virtual uint8_t _GetHandshakeResponseLength() override
    {
        return sizeof(_handshake_response);
    };

    virtual const uint8_t *_GetPacketRequest() override
    {
        return _packet_request;
    };

    virtual uint8_t _GetPacketRequestLength() override
    {
        return sizeof(_packet_request);
    };

    /// @brief Разбирает пакет с начала буфера, заполняя данные в структуру мотора
    /// @param buffer Буфер с пакетом
    /// @param length Длина данных в буфере
    /// @return число байт — длина обработанного пакета, даже если из него никакие данные извлечены не были (но он был валидный).
    ///         '0' — если обработка не удалась (пакет не валидный или в буфере не пакет вовсе).
    virtual uint8_t _ParsePacket(motor_data_t &motor_data, uint8_t *buffer, uint8_t length) override
    {
        if (!IsValidPacket(buffer, length))
            return 0;

        // TODO: сделать тут разбор пакетов для протокола 2022 года!
        switch (buffer[1])
        {
        case 0x01:
            motor_data.voltage = *(uint16_t *)&buffer[2];
            motor_data.current = *(uint16_t *)&buffer[4];
            break;

        default:
            break;
        }

        return _packet_length;
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

        // swap endian
        union bit16_data
        {
            uint16_t u16;
            struct
            {
                uint8_t low;
                uint8_t high;
            } bytes;
        };
        bit16_data data16;
        data16.bytes.low = buffer[1];
        data16.bytes.high = buffer[0];

        // return crc == *(uint16_t *)buffer;
        return crc == data16.u16;
    };
};

/*********************************************************************************************************
 *********************************************************************************************************/
class FardriverParser2023 : public MotorParserInterface
{
public:
    /// @brief Вызывается, если получен корректный хендшейк
    /// @param manager Переданный по ссылке менеджер моторов
    /// @param motor_idx Индекс мотора, с которым работаем
    virtual void OnHandshakeReceived(MotorManagerInterface &manager, uint8_t motor_idx, uint8_t *buffer, uint8_t length) override
    {
        // отправляем ответ на хендшейк
        manager.TXEvent(motor_idx, _handshake_response, sizeof(_handshake_response));
    };

    /// @brief Вызывается, если получен корректный пакет.
    /// @param manager Переданный по ссылке менеджер моторов
    /// @param motor_idx Индекс мотора, с которым работаем
    virtual void OnPacketReceived(MotorManagerInterface &manager, uint8_t motor_idx, uint8_t *buffer, uint8_t length) override
    {
        // запрос данных не требуется
        return;
    };

private:
    // static constexpr uint8_t _handshake_start_bytes_count = 3;
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

    /// @brief Возвращает указатель на ожидаемый запрос хендшейка от драйвера
    /// @return Указатель на буфер, содержащий запрос хендшейка, или nullptr, если запроса нет
    virtual const uint8_t *_GetHandshakeRequest()
    {
        return _handshake_request;
    };

    /// @brief Возвращает длину в байтах ожидаемого от драйвера запроса хендшейка
    /// @return Количество байт в запросе хендшейка или '0', если ответа нет
    virtual uint8_t _GetHandshakeRequestLength()
    {
        return sizeof(_handshake_request);
    };

    /// @brief Возвращает указатель на ответ на хендшейк
    /// @return Указатель на буфер, содержащий ответ на хендшейк, или nullptr, если ответа нет
    virtual const uint8_t *_GetHandshakeResponse() override
    {
        return _handshake_response;
    };

    /// @brief Возвращает длину в байтах ответа на хендшейк
    /// @return Количество байт в ответе на хендшейк или '0', если ответа нет
    virtual uint8_t _GetHandshakeResponseLength() override
    {
        return sizeof(_handshake_response);
    };

    /// @brief Разбирает пакет с начала буфера, заполняя данные в структуру мотора
    /// @param buffer Буфер с пакетом
    /// @param length Длина данных в буфере
    /// @return число байт — длина обработанного пакета, даже если из него никакие данные извлечены не были (но он был валидный).
    ///         '0' — если обработка не удалась (пакет не валидный или в буфере не пакет вовсе).
    virtual uint8_t _ParsePacket(motor_data_t &motor_data, uint8_t *buffer, uint8_t length) override
    {
        if (!IsValidPacket(buffer, length))
            return 0;

        // TODO: сделать тут разбор пакетов для протокола 2022 года!
        switch (buffer[1])
        {
        case 0xA4:
            motor_data.voltage = *(uint16_t *)&buffer[2];
            motor_data.current = *(uint16_t *)&buffer[4];
            break;

        default:
            break;
        }

        return _packet_length;
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
private:
    static constexpr uint8_t _max_motors = 2;
    static constexpr uint8_t _rx_buffer_size = 128;

    typedef struct
    {
        uint8_t motor_idx;                  // индекс мотора
        uint8_t rx_buffer[_rx_buffer_size]; // Приёмный буфер
        uint8_t rx_data_size;               // Длина данных в буфере, данные всегда начинаются с начала буфера

        MotorParserInterface *parser;

        motor_data_t motor_data; // распарсенные данные по мотору, только те, что нам нужны
    } motor_t;

    motor_t _motors[_max_motors];

    MotorParserInterface *_first_parser = nullptr;

    tx_callback_t _tx_callback = nullptr;
    uint32_t _last_time = 0;

public:
    MotorManager(tx_callback_t tx_callback)
    {
        _tx_callback = tx_callback;
        for (uint8_t i = 0; i < _max_motors; i++)
        {
            _motors[i].motor_idx = i;
            _motors[i].parser = nullptr;
            memset(_motors[i].rx_buffer, 0, _rx_buffer_size);
            _motors[i].rx_data_size = 0;
            memset(&_motors[i].motor_data, 0, sizeof(motor_data_t));
        }

        _first_parser = new FardriverParser2023;
        _first_parser->SetNextParser(new FardriverParser2022);
    }

    ~MotorManager()
    {
        _first_parser->DeleteNextParser();
        delete _first_parser;

        for (motor_t &motor : _motors)
        {
            motor.parser = nullptr;
        }
    };

    virtual void RXEvent(uint8_t motor_idx, uint8_t *buffer, uint8_t length) override
    {
        if (motor_idx >= _max_motors)
            return;

        memcpy(_motors[motor_idx].rx_buffer, buffer, length);
        _motors[motor_idx].rx_data_size = length;
    }

    virtual void TXEvent(uint8_t motor_idx, const uint8_t *buffer, uint8_t length) override
    {
        if (motor_idx >= _max_motors || _tx_callback == nullptr)
            return;

        _tx_callback(motor_idx, buffer, length);
    }

    virtual void Processing(uint32_t time) override
    {
        if (time - _last_time <= 1)
            return;

        _last_time = time;

        for (motor_t &motor : _motors)
        {
            if (motor.rx_data_size == 0)
                continue;

            // если данные ждут разбора...
            if (motor.parser == nullptr)
            {
                // последний успешно использованный парсер отсутствует, пробуем разбор начиная с первого в цепочке
                motor.parser = _first_parser->ParsePacket(*this, motor.motor_idx, motor.motor_data, motor.rx_buffer, motor.rx_data_size);
            }
            else
            {
                // есть последний успешно использованный парсер, пробуем использовать его для разбора
                motor.parser = motor.parser->ParsePacket(*this, motor.motor_idx, motor.motor_data, motor.rx_buffer, motor.rx_data_size);
            }
        }
    }

    virtual void DeleteNFirstBytesFromBuffer(uint8_t motor_idx, uint8_t number_of_bytes) override
    {
        if (number_of_bytes == 0)
            return;

        if (number_of_bytes > _rx_buffer_size)
            number_of_bytes = _rx_buffer_size;

        if (number_of_bytes == _rx_buffer_size || number_of_bytes >= _motors[motor_idx].rx_data_size)
        {
            _motors[motor_idx].rx_data_size = 0;
            return;
        }

        _motors[motor_idx].rx_data_size = _motors[motor_idx].rx_data_size - number_of_bytes;
        memmove(_motors[motor_idx].rx_buffer, _motors[motor_idx].rx_buffer + number_of_bytes, _motors[motor_idx].rx_data_size);
    };

    virtual motor_data_t *GetMotorData(uint8_t motor_idx) override
    {
        if (motor_idx >= _max_motors)
            return nullptr;

        return &_motors[motor_idx].motor_data;
    };
};
