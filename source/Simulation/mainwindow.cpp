#include "Simulation/mainwindow.h"
#include "Simulation/CommonSimParams.hpp"
#include "./ui_mainwindow.h"

MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent), ui(new Ui::MainWindow)
{
	ui->setupUi(this);
	setWindowIcon(QIcon(":/icons/icon.ico"));
	_leSim = new Simulation();

	radiusCurve = new QCPCurve(ui->plot->xAxis, ui->plot->yAxis);
	auto radCrvCasted = static_cast<QCPCurve*>(radiusCurve);
	QPen radCurvePen(QPen(QColor("green")));

	radCurvePen.setWidth(4);
	radCrvCasted->setPen(radCurvePen);
	radCrvCasted->setVisible(false);
	radCrvCasted->setBrush(QBrush(QColor(0, 128, 0, 64)));
	radCrvCasted->setName("Missile Proximity Radius");
	radCrvCasted->setAntialiased(true);

	ui->plot->legend->setVisible(true);

	ui->plot->addGraph()->setName("Missile");
	ui->plot->addGraph()->setName("Target");
	
	ui->plot->graph(0)->setScatterStyle(QCPScatterStyle::ssDisc);
	ui->plot->graph(0)->setLineStyle(QCPGraph::lsNone);
	ui->plot->graph(0)->setPen(QPen(QColor("red")));
	
	ui->plot->graph(1)->setScatterStyle(QCPScatterStyle::ssDisc);
	ui->plot->graph(1)->setLineStyle(QCPGraph::lsNone);
	ui->plot->graph(1)->setPen(QPen(QColor("blue")));

	ui->plot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);

	std::thread dataPrepThread([&]{ prepareHitRadData(); });
	dataPrepThread.detach();
}

MainWindow::~MainWindow()
{
	delete ui;
	delete _leSim;
}

void MainWindow::on_startSimBtn_clicked()
{
	auto leTgt = _leSim->getTarget();
	auto leMsl = _leSim->getMissile();
	
	_leSim->setFileOutputNeededTo(ui->fileOCheckBox->isChecked());
	_leSim->restoreSimState();
	
	leTgt->setCoords(0, ui->distanceSpinBox->value());
	leTgt->setVelocity(0, -ui->tgtSpeedSpinBox->value());
	leTgt->setEvasiveActionState(ui->tgtEvActCheckBox->isChecked());
	
	leMsl->setCoords(0, 0);
	leMsl->setVelocity(0, ui->mslSpeedSpinBox->value());
	leMsl->setNavConstant(ui->navConstDoubleSpinBox->value());

	ui->outputLabel->clear();
	ui->outputLabel->setText(tr("Simulation's running; please wait"));
	ui->outputLabel->setStyleSheet("QLabel { color: black; text-align: center; }");

	if (radiusCurve) static_cast<QCPCurve*>(radiusCurve)->setVisible(false);

	float tSinceReplot = 0;
	simFinished = false;

	mslX.clear(); mslY.clear();
	tgtX.clear(); tgtY.clear();
	tgtX.append(_leSim->getTarget()->getX());
	tgtY.append(_leSim->getTarget()->getY());
	mslX.append(_leSim->getMissile()->getX());
	mslY.append(_leSim->getMissile()->getY());
	plot();
	
	std::thread simThread([&]{ runSim(); });
	simThread.detach();

	while (!simFinished)
	{
		if (tSinceReplot >= 0.5)
		{
			tSinceReplot = 0;
			plot();
		}

		tSinceReplot += SIM_RESOLUTION;
	}

	if (_leSim->mslWithinTgtHitRadius())
	{
		ui->outputLabel->setText(tr("Simulation's been stopped: the missile has reached the target"));
		ui->outputLabel->setStyleSheet("QLabel { color: green; text-align: center; }");
	}
	else if (!_leSim->mslSpeedMoreThanTgtSpeed())
	{
		ui->outputLabel->setText(tr("Simulation's been stopped: the missile's velocity has fallen below the target's"));
		ui->outputLabel->setStyleSheet("QLabel { color: red; text-align: center; }");
	}

	plot(true);
}

void MainWindow::on_resetSimBtn_clicked()
{
	mslX.clear(); mslY.clear();
	tgtX.clear(); tgtY.clear();
	ui->outputLabel->clear();
	_leSim->restoreSimState();
	if (radiusCurve) static_cast<QCPCurve*>(radiusCurve)->setVisible(false);
	simFinished = false;

	plot();
}

void MainWindow::plot(bool doFilter)
{
	if (doFilter)
	{
		auto filterData = [&](QVector<double>& keyVec, QVector<double>& valVec)
		{
			auto filterInterval = _leSim->getMissile()->getProxyRadius() * 5;
			
			if (keyVec.size() <= 1)
				return;

			for (auto i = 5; i < keyVec.size() - 5; ++i) // don't filter some initial and some final points
			{
				auto pointDist = sqrt(pow(keyVec.at(i) - keyVec.at(i - 1), 2) + pow(valVec.at(i) - valVec.at(i - 1), 2));
				if (pointDist < filterInterval)
				{
					keyVec.remove(i);
					valVec.remove(i);
					i--;
				}
			}
		};

		filterData(mslX, mslY);
		filterData(tgtX, tgtY);
	}

	ui->plot->graph(0)->setData(mslX, mslY);
	ui->plot->graph(1)->setData(tgtX, tgtY);
	ui->plot->rescaleAxes();
	ui->plot->xAxis->setScaleRatio(ui->plot->yAxis, 1);
	ui->plot->yAxis->setScaleRatio(ui->plot->xAxis, 1);
	ui->plot->graph(0)->setAdaptiveSampling(true);
	ui->plot->graph(1)->setAdaptiveSampling(true);

	if (doFilter && _leSim)
	{
		auto mslFinalX = mslX.last();
		auto mslFinalY = mslY.last();
		QVector<double> xCoords, yCoords;
		
		xCoords.clear(); yCoords.clear();
		
		for (auto coordX : hitRadX)
			xCoords.append(coordX + mslFinalX);
		
		for (auto coordY : hitRadY)
			yCoords.append(coordY + mslFinalY);
		
		auto radCrvCasted = static_cast<QCPCurve*>(radiusCurve);
		radCrvCasted->setData(xCoords, yCoords);
		radCrvCasted->setVisible(true);
	}

	ui->plot->replot();
	ui->plot->update();
}

void MainWindow::runSim()
{
	while (!simFinished)
	{
		_leSim->iterate();

		tgtX.append(_leSim->getTarget()->getX());
		tgtY.append(_leSim->getTarget()->getY());
		mslX.append(_leSim->getMissile()->getX());
		mslY.append(_leSim->getMissile()->getY());

		simFinished = _leSim->mslWithinTgtHitRadius() || !_leSim->mslSpeedMoreThanTgtSpeed();
	}
}

void MainWindow::prepareHitRadData()
{
	const static double degreesPerStep{ 0.5 };
	QVector2D radVector{ float(_leSim->getMslProxyRadius()), 0.f };
	double currentAngle{ 0 };

	hitRadX.append(radVector.x());
	hitRadY.append(radVector.y());

	while (currentAngle <= 360)
	{
		currentAngle += degreesPerStep;
		rotateVec(degreesPerStep, radVector, false);

		hitRadX.append(radVector.x());
		hitRadY.append(radVector.y());
	}
}

void MainWindow::_createLangMenu(void)
{
	auto langGroup = new QActionGroup(ui->menuLanguage);
	auto defaultLocale = QLocale::system().name();

	langGroup->setExclusive(true);

	connect(langGroup, SIGNAL(triggered(QAction*)), this, SLOT(slotLangChanged(QAction*)));

	defaultLocale.truncate(defaultLocale.lastIndexOf('_'));

	langPath = QApplication::applicationDirPath() + "/translations";

	QDir langDir(langPath);
	QStringList fileNames = langDir.entryList(QStringList("qt_*.qm"));

	for (auto fName : fileNames)
	{
		QString locale = fName;
		locale.truncate(locale.lastIndexOf('.'));
		locale.remove(0, locale.lastIndexOf('_'));

		QString lang = QLocale::languageToString(QLocale(locale).language());
		auto action = new QAction(lang, this);
		action->setCheckable(true);
		action->setData(locale);

		ui->menuLanguage->addAction(action);
		langGroup->addAction(action);

		if (defaultLocale == locale)
			action->setChecked(true);
	}
}

void MainWindow::slotLangChanged(QAction* action)
{
	if (action)
		_loadLanguage(action->data().toString());
}

void MainWindow::_changeEvent(QEvent* event)
{
	if (event)
	{
		switch (event->type())
		{
			case QEvent::LanguageChange:
			{
				ui->retranslateUi(this);
				break;
			}
			case QEvent::LocaleChange:
			{
				QString locale = QLocale::system().name();
				locale.truncate(locale.lastIndexOf('_'));
				_loadLanguage(locale);
			}
			default:
				break;
		}
	}

	QMainWindow::changeEvent(event);
}

void switchTranslator(QTranslator& tr, const QString& filename)
{
	qApp->removeTranslator(&tr);

	QString path = QApplication::applicationDirPath() + "/translations/";
	if (tr.load(path + filename))
		qApp->installTranslator(&tr);
}

void MainWindow::_loadLanguage(const QString& langID)
{
	if (currLang != langID)
	{
		currLang = langID;

		QLocale locale = QLocale(currLang);
		QLocale::setDefault(locale);
		QString langName = QLocale::languageToString(locale.language());

		switchTranslator(leTrans, QString("qt_%1.qm").arg(langID));
		switchTranslator(leTransQt, QString("qt_%1.qm").arg(langID));

	}
}
