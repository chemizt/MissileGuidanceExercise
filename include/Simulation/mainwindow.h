#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <thread>
#include <cmath>

#include "Simulation/simulation.hpp"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
	Q_OBJECT

	public:
		MainWindow(QWidget* parent = nullptr);
		~MainWindow();

		void runSim();
		void prepareHitRadData();
	private slots:
		void on_startSimBtn_clicked();
		void on_resetSimBtn_clicked();

private:
		Ui::MainWindow* ui;
		QVector<double> mslX, mslY, tgtX, tgtY, hitRadX, hitRadY;
		void* radiusCurve{ nullptr };
		bool simFinished{ false };
		void plot(bool doFilter = false);
		Simulation* _leSim{ nullptr };

};
#endif // MAINWINDOW_H
