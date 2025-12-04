#ifndef UDP2EC2PROTOCOL_H
#define UDP2EC2PROTOCOL_H

#include <QObject>
#include <QUdpSocket>
#include <QTimer>
#include <QHostAddress>
#include <QVector>
class Controller;
struct SectorDataSet {
    float drisa_f = 0.0f;
    float drisa_l = 0.0f;
    float irtifa_f = 0.0f;
    float irtifa_l = 0.0f;
};
enum TYPES{
    SECTOR_OBSTACLE,
    SECTOR_MOTOR_CUTOUT
};

class UDP2EC2Protocol : public QObject
{
    Q_OBJECT

public:
    UDP2EC2Protocol();
    ~UDP2EC2Protocol() = default;
    void init(Controller *_ctrl);
    // Okuma/Yazma başlatma
    void startReading(const int &_sector_type);
    void startWriting(const QVector<SectorDataSet> &dataList, const int &_sector_type);

    // Veri erişimi
    QVector<SectorDataSet> getAllData() const { return m_dataList; }
    SectorDataSet getData(int index) const;
    QVector<SectorDataSet> m_writeDataList;
    // Durum kontrolü
    bool isReading() const { return m_isReading; }
    bool isWriting() const { return m_isWriting; }
      bool parseResponsePacket(const QByteArray &packet);
signals:
    void dataReadComplete();
    void dataWriteComplete();
    void dataSetReceived(int index, const SectorDataSet &data);
    void errorOccurred(const QString &error);
    void statusMessage(const QString &message);

private slots:
    // void onReadyRead();
    void onRequestTimeout();
    void onWriteTimeout();

private:
    // Paket oluşturma
    QByteArray createRequestPacket(int dataIndex);
    QByteArray createWritePacket(int dataIndex, const SectorDataSet &data);

    // Paket işleme

    quint8 calculateChecksum(const QByteArray &data);

    // Gönderme fonksiyonları
    void sendRequestPacket();
    void sendWritePacket();

    // UDP - Stack-based, parent ownership ile
    QUdpSocket m_socket;
    QHostAddress m_targetIp;
    quint16 m_targetPort;

    // Timer'lar - Stack-based, parent ownership ile
    QTimer m_requestTimer;
    QTimer m_writeTimer;

    // Veri yapıları
    QVector<SectorDataSet> m_dataList;


    // Durum değişkenleri
    int m_totalCount;
    int m_currentReadIndex;
    int m_currentWriteIndex;
    int m_readRetryCount;
    int m_writeRetryCount;
    bool m_isReading;
    bool m_isWriting;

    // Sabitler
    static constexpr int MAX_RETRIES = 5;
    static constexpr int RETRY_DELAY_MS = 2000;
    static constexpr int PACKET_SIZE = 36;

    Controller *m_controller;

    int m_currentReadSector = SECTOR_OBSTACLE;
    int m_currentWriteSector = SECTOR_OBSTACLE;
};

#endif // UDP2EC2PROTOCOL_H
