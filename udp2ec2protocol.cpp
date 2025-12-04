
// ============================================================================
// CPP IMPLEMENTATION
// ============================================================================

#include <QDebug>
#include <QtEndian>
#include "udp2ec2protocol.h"
#include "controller.h"
#include <QHostAddress>
UDP2EC2Protocol::UDP2EC2Protocol()
    : m_requestTimer(this)  // Parent ownership
    , m_writeTimer(this)    // Parent ownership
    , m_totalCount(0)
    , m_currentReadIndex(0)
    , m_currentWriteIndex(0)
    , m_readRetryCount(0)
    , m_writeRetryCount(0)
    , m_isReading(false)
    , m_isWriting(false)
{
    // Socket ayarları
    // m_socket.bind(QHostAddress::Any, listenPort);
    // connect(&m_socket, &QUdpSocket::readyRead, this, &UDP2EC2Protocol::onReadyRead);

    // Request timer ayarları
    m_requestTimer.setInterval(RETRY_DELAY_MS);
    connect(&m_requestTimer, &QTimer::timeout, this, &UDP2EC2Protocol::onRequestTimeout);

    // Write timer ayarları
    m_writeTimer.setInterval(RETRY_DELAY_MS);
    connect(&m_writeTimer, &QTimer::timeout, this, &UDP2EC2Protocol::onWriteTimeout);

    emit statusMessage("UDP 2EC2 Protokolü başlatıldı");

    qDebug()<<__FUNCTION__<<":Started";

}

void UDP2EC2Protocol::init(Controller *_ctrl)
{
    m_controller = _ctrl;
}

quint8 UDP2EC2Protocol::calculateChecksum(const QByteArray &data)
{
    quint8 checksum = 0;
    for (int i = 0; i < data.size(); i++) {
        checksum += static_cast<quint8>(data[i]);
    }
    return checksum;
}

QByteArray UDP2EC2Protocol::createRequestPacket(int dataIndex)
{
    QByteArray packet(PACKET_SIZE, 0);

    // Header
    packet[0] = 0x0C;  // TX ID
    packet[1] = 0x03;  // RX ID
    packet[2] = 0x00;  // FUNC - Request

    // DATA 1: Type (1 = Motor CutOut)
    packet[3] = m_currentReadSector;

    // DATA 2: Total Count
    packet[4] = static_cast<char>(m_totalCount);

    // DATA 3: Read Count (hangi data set'i istiyoruz)
    packet[5] = static_cast<char>(dataIndex);

    // Checksum (35 byte'ın checksumu - son byte hariç)
    packet[35] = calculateChecksum(packet.left(35));

    return packet;
}

QByteArray UDP2EC2Protocol::createWritePacket(int dataIndex, const SectorDataSet &data)
{
    QByteArray packet(PACKET_SIZE, 0);

    // Header
    packet[0] = 0x0C;  // TX ID
    packet[1] = 0x03;  // RX ID
    packet[2] = 0x01;  // FUNC - Write

    // DATA 1: Type
    packet[3] = 0x00;  // 0 Obstacle 1 Motor CutOut

    // DATA 2: Total Count
    packet[4] = static_cast<char>(m_writeDataList.size());

    // DATA 3: Current Count (kaçıncı data set)
    packet[5] = static_cast<char>(dataIndex);

    // Float'ları byte array'e çevir (Little Endian)
    union {
        float f;
        quint32 i;
    } converter;

    // DATA 6-9: Drisa_F
    converter.f = data.drisa_f;
    quint32 leValue = qToLittleEndian(converter.i);
    memcpy(packet.data() + 6, &leValue, sizeof(float));

    // DATA 10-13: Drisa_L
    converter.f = data.drisa_l;
    leValue = qToLittleEndian(converter.i);
    memcpy(packet.data() + 10, &leValue, sizeof(float));

    // DATA 14-17: Irtifa_F
    converter.f = data.irtifa_f;
    leValue = qToLittleEndian(converter.i);
    memcpy(packet.data() + 14, &leValue, sizeof(float));

    // DATA 18-21: Irtifa_L
    converter.f = data.irtifa_l;
    leValue = qToLittleEndian(converter.i);
    memcpy(packet.data() + 18, &leValue, sizeof(float));

    // Checksum (35 byte'ın checksumu)
    packet[35] = calculateChecksum(packet.left(35));

    return packet;
}

bool UDP2EC2Protocol::parseResponsePacket(const QByteArray &packet)
{
    if (packet.size() != PACKET_SIZE) {
        emit errorOccurred("Hatalı paket boyutu");
        return false;
    }

    // Checksum kontrolü (ilk 35 byte'ın checksumu)
    quint8 receivedChecksum = static_cast<quint8>(packet[35]);
    quint8 calculatedChecksum = calculateChecksum(packet.left(35));

    if (receivedChecksum != calculatedChecksum) {
        emit errorOccurred(QString("Checksum hatası! Beklenen: %1, Gelen: %2")
                               .arg(calculatedChecksum).arg(receivedChecksum));
        return false;
    }

    quint8 func = static_cast<quint8>(packet[2]);
    quint8 dataType = static_cast<quint8>(packet[3]);
    quint8 totalCount = static_cast<quint8>(packet[4]);
    quint8 currentIndex = static_cast<quint8>(packet[5]);
    if(totalCount == 0)
    {
        emit statusMessage(QString("Toplam dataset sayısı 0 olarak belirtildi. İşlem iptal ediliyor."));
        m_currentReadIndex++;
        sendRequestPacket();
        return true;
    }
    // Write işlemi onayı (FUNC = 0x01)
    if (func == 0x01 && m_isWriting) {
        if (currentIndex == m_currentWriteIndex) {
            emit statusMessage(QString("Data set %1 yazıldı ve onaylandı").arg(currentIndex));
            m_currentWriteIndex++;
            m_writeRetryCount = 0;

            if (m_currentWriteIndex >= m_writeDataList.size()) {
                m_isWriting = false;
                m_writeTimer.stop();
                emit statusMessage("Tüm data setler yazıldı!");
                emit dataWriteComplete();
            } else {
                // Bir sonraki data set'i hemen gönder
                sendWritePacket();
            }
            return true;
        }
    }

    // Read işlemi yanıtı (FUNC = 0x00)
    if (func == 0x00 && m_isReading) {
        // İlk pakette total count'u öğren
        if (m_totalCount == 0) {
            m_totalCount = totalCount;
            m_dataList.resize(m_totalCount);
            emit statusMessage(QString("Toplam %1 data set olduğu öğrenildi").arg(m_totalCount));
        }

        if (currentIndex == m_currentReadIndex) {
            SectorDataSet data;

            // Float verileri oku (Little Endian)
            union {
                float f;
                quint32 i;
            } converter;

            quint32 leValue;

            // Drisa_F (DATA 6-9)
            memcpy(&leValue, packet.data() + 6, sizeof(float));
            converter.i = qFromBigEndian(leValue);
            data.drisa_f = converter.f;

            // Drisa_L (DATA 10-13)
            memcpy(&leValue, packet.data() + 10, sizeof(float));
            converter.i = qFromBigEndian(leValue);
            data.drisa_l = converter.f;

            // Irtifa_F (DATA 14-17)
            memcpy(&leValue, packet.data() + 14, sizeof(float));
            converter.i = qFromBigEndian(leValue);
            data.irtifa_f = converter.f;

            // Irtifa_L (DATA 18-21)
            memcpy(&leValue, packet.data() + 18, sizeof(float));
            converter.i = qFromBigEndian(leValue);
            data.irtifa_l = converter.f;

            // Veriyi kaydet
            m_dataList[currentIndex] = data;

            emit statusMessage(QString("Data set %1 alındı: Drisa_F=%2, Drisa_L=%3, Irtifa_F=%4, Irtifa_L=%5")
                                   .arg(currentIndex)
                                   .arg(data.drisa_f)
                                   .arg(data.drisa_l)
                                   .arg(data.irtifa_f)
                                   .arg(data.irtifa_l));

            emit dataSetReceived(currentIndex, data);

            m_currentReadIndex++;
            m_readRetryCount = 0;

            if (m_currentReadIndex > m_totalCount) {
                m_isReading = false;
                m_requestTimer.stop();
                emit statusMessage("Tüm data setler okundu!");
                emit dataReadComplete();
            } else {
                // Bir sonraki data set'i hemen iste
                sendRequestPacket();
            }
            return true;
        }
    }

    return false;
}

void UDP2EC2Protocol::sendRequestPacket()
{
    QByteArray packet = createRequestPacket(m_currentReadIndex);
    if(m_currentReadSector != SECTOR_OBSTACLE && m_currentReadSector != SECTOR_MOTOR_CUTOUT)
    {
        emit statusMessage(QString("Okuma için ayarlanan sektor doğru değil. Böyle bir sektör yok."));
        return;
    }
    m_controller->getClient()->writeData(packet,QHostAddress("192.168.10.15"));
    //m_socket.writeDatagram(packet, m_targetIp, m_targetPort);
    emit statusMessage(QString("Data set %1 istendi (deneme %2)")
                           .arg(m_currentReadIndex)
                           .arg(m_readRetryCount + 1));
}

void UDP2EC2Protocol::sendWritePacket()
{
    if (m_currentWriteIndex < m_writeDataList.size()) {
        QByteArray packet = createWritePacket(m_currentWriteIndex, m_writeDataList[m_currentWriteIndex]);
        m_socket.writeDatagram(packet, m_targetIp, m_targetPort);
        emit statusMessage(QString("Data set %1 gönderildi (deneme %2)")
                               .arg(m_currentWriteIndex)
                               .arg(m_writeRetryCount + 1));
    }
}


void UDP2EC2Protocol::onRequestTimeout()
{
    m_readRetryCount++;

    if (m_readRetryCount >= MAX_RETRIES) {
        m_isReading = false;
        m_requestTimer.stop();
        emit errorOccurred(QString("Data set %1 alınamadı! Maksimum deneme aşıldı.")
                               .arg(m_currentReadIndex));
        return;
    }

    sendRequestPacket();
}

void UDP2EC2Protocol::onWriteTimeout()
{
    m_writeRetryCount++;

    if (m_writeRetryCount >= MAX_RETRIES) {
        m_isWriting = false;
        m_writeTimer.stop();
        emit errorOccurred(QString("Data set %1 yazılamadı! Maksimum deneme aşıldı.")
                               .arg(m_currentWriteIndex));
        return;
    }

    sendWritePacket();
}

void UDP2EC2Protocol::startReading(const int &_sector_type)
{

    if (m_isReading) {
        emit statusMessage("Okuma işlemi zaten devam ediyor");
        return;
    }
    if(_sector_type > 1){
        emit statusMessage("Okunması için seçilen sektör yanlış. Böyle bir sektör türü yok. 0:Obstacle - 1:MotorCutout");
        return;
    }
    m_currentReadSector = _sector_type;
    m_totalCount = 0;  // Total count'u ilk paketten öğreneceğiz
    m_currentReadIndex = 0;
    m_readRetryCount = 0;
    m_isReading = true;
    m_dataList.clear();
    emit statusMessage
        (QString((m_currentReadIndex==0 ? "Obstacle ":"Motor Cutout"))+
                       "Veri okuma başlatıldı");
    sendRequestPacket();
    m_requestTimer.start();
}

void UDP2EC2Protocol::startWriting(const QVector<SectorDataSet> &dataList,const int &_sector_type)
{
    if (m_isWriting) {
        emit statusMessage("Yazma işlemi zaten devam ediyor");
        return;
    }

    if (dataList.isEmpty()) {
        emit errorOccurred("Yazılacak veri yok!");
        return;
    }
    if(_sector_type > 1){
        emit statusMessage("Yazılması için seçilen sektör numarası yanlış.");
        return;
    }
    m_currentWriteSector = _sector_type;
    m_writeDataList = dataList;
    m_currentWriteIndex = 0;
    m_writeRetryCount = 0;
    m_isWriting = true;

    emit statusMessage(QString("Veri yazma başlatıldı (%1 data set)").arg(dataList.size()));

    sendWritePacket();
    m_writeTimer.start();
}

SectorDataSet UDP2EC2Protocol::getData(int index) const
{
    if (index >= 0 && index < m_dataList.size()) {
        return m_dataList[index];
    }
    return SectorDataSet();
}
// ============================================================================
// KULLANIM ÖRNEĞİ - main.cpp
// ============================================================================

/*
#include <QCoreApplication>
#include "udp2ec2protocol.h"

int main(int argc, char *argv[])
{
    QCoreApplication app(argc, argv);

    // Protokol oluştur - Stack-based, parent ownership
    UDP2EC2Protocol protocol("192.168.1.100", 5000, 5001, &app);

    // Signal bağlantıları
    QObject::connect(&protocol, &UDP2EC2Protocol::statusMessage, [](const QString &msg) {
        qDebug() << "Status:" << msg;
    });

    QObject::connect(&protocol, &UDP2EC2Protocol::errorOccurred, [](const QString &error) {
        qDebug() << "Error:" << error;
    });

    QObject::connect(&protocol, &UDP2EC2Protocol::dataSetReceived, [](int index, const MotorDataSet &data) {
        qDebug() << QString("Data set %1 geldi:").arg(index);
        qDebug() << "  Drisa_F:" << data.drisa_f;
        qDebug() << "  Drisa_L:" << data.drisa_l;
        qDebug() << "  Irtifa_F:" << data.irtifa_f;
        qDebug() << "  Irtifa_L:" << data.irtifa_l;
    });

    QObject::connect(&protocol, &UDP2EC2Protocol::dataReadComplete, [&protocol]() {
        qDebug() << "Okuma tamamlandı!";
        qDebug() << "Toplam data set sayısı:" << protocol.getAllData().size();
    });

    QObject::connect(&protocol, &UDP2EC2Protocol::dataWriteComplete, []() {
        qDebug() << "Yazma tamamlandı!";
    });

    // Program başladığında verileri oku
    protocol.startReading();

    // 5 saniye sonra veri yaz
    QTimer::singleShot(5000, [&protocol]() {
        qDebug() << "\n=== VERİ YAZMA BAŞLIYOR ===";

        QVector<MotorDataSet> dataToWrite;

        MotorDataSet data1{12.5f, 15.7f, 100.0f, 105.3f};
        dataToWrite.append(data1);

        MotorDataSet data2{20.0f, 22.5f, 200.0f, 205.0f};
        dataToWrite.append(data2);

        protocol.startWriting(dataToWrite);
    });

    return app.exec();
}
*/
