#pragma once

#include <QObject>

// QML-exposed model bound to the gizmo view; owned by GizmoController.
class SensorData : public QObject
{
	Q_OBJECT
	Q_PROPERTY(qreal pitch READ pitch NOTIFY pitchChanged)
	Q_PROPERTY(qreal yaw READ yaw NOTIFY yawChanged)
	Q_PROPERTY(qreal roll READ roll NOTIFY rollChanged)

public:
	qreal pitch() const { return m_pitch; }
	qreal yaw() const { return m_yaw; }
	qreal roll() const { return m_roll; }

	void update(qreal p, qreal y, qreal r)
	{
		m_pitch = p;
		m_yaw	= y;
		m_roll	= r;
		emit pitchChanged();
		emit yawChanged();
		emit rollChanged();
	}

signals:
	void pitchChanged();
	void yawChanged();
	void rollChanged();

private:
	qreal m_pitch = 0, m_yaw = 0, m_roll = 0;
};
