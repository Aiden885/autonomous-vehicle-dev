/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 5.14.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralWidget;
    QLabel *label_port;
    QLabel *label_latlon;
    QLabel *label_azi;
    QLabel *label_spe;
    QLabel *label_spe2;
    QLabel *label_RTKdata;
    QPushButton *pushButton;
    QLabel *label_show;
    QPushButton *pushButton_record;
    QMenuBar *menuBar;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(726, 406);
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
        label_port = new QLabel(centralWidget);
        label_port->setObjectName(QString::fromUtf8("label_port"));
        label_port->setGeometry(QRect(10, 10, 691, 32));
        label_latlon = new QLabel(centralWidget);
        label_latlon->setObjectName(QString::fromUtf8("label_latlon"));
        label_latlon->setGeometry(QRect(10, 50, 691, 32));
        label_azi = new QLabel(centralWidget);
        label_azi->setObjectName(QString::fromUtf8("label_azi"));
        label_azi->setGeometry(QRect(10, 90, 691, 32));
        label_spe = new QLabel(centralWidget);
        label_spe->setObjectName(QString::fromUtf8("label_spe"));
        label_spe->setGeometry(QRect(10, 130, 691, 32));
        label_spe2 = new QLabel(centralWidget);
        label_spe2->setObjectName(QString::fromUtf8("label_spe2"));
        label_spe2->setGeometry(QRect(10, 170, 691, 32));
        label_RTKdata = new QLabel(centralWidget);
        label_RTKdata->setObjectName(QString::fromUtf8("label_RTKdata"));
        label_RTKdata->setGeometry(QRect(160, 250, 541, 32));
        pushButton = new QPushButton(centralWidget);
        pushButton->setObjectName(QString::fromUtf8("pushButton"));
        pushButton->setGeometry(QRect(10, 250, 131, 46));
        label_show = new QLabel(centralWidget);
        label_show->setObjectName(QString::fromUtf8("label_show"));
        label_show->setGeometry(QRect(10, 210, 691, 32));
        pushButton_record = new QPushButton(centralWidget);
        pushButton_record->setObjectName(QString::fromUtf8("pushButton_record"));
        pushButton_record->setGeometry(QRect(610, 240, 91, 51));
        MainWindow->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(MainWindow);
        menuBar->setObjectName(QString::fromUtf8("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 726, 37));
        MainWindow->setMenuBar(menuBar);
        mainToolBar = new QToolBar(MainWindow);
        mainToolBar->setObjectName(QString::fromUtf8("mainToolBar"));
        MainWindow->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(MainWindow);
        statusBar->setObjectName(QString::fromUtf8("statusBar"));
        MainWindow->setStatusBar(statusBar);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QCoreApplication::translate("MainWindow", "MainWindow", nullptr));
        label_port->setText(QCoreApplication::translate("MainWindow", "TextLabel", nullptr));
        label_latlon->setText(QCoreApplication::translate("MainWindow", "TextLabel", nullptr));
        label_azi->setText(QCoreApplication::translate("MainWindow", "TextLabel", nullptr));
        label_spe->setText(QCoreApplication::translate("MainWindow", "TextLabel", nullptr));
        label_spe2->setText(QCoreApplication::translate("MainWindow", "TextLabel", nullptr));
        label_RTKdata->setText(QCoreApplication::translate("MainWindow", "TextLabel", nullptr));
        pushButton->setText(QCoreApplication::translate("MainWindow", "\344\270\262\345\217\243\347\233\264\350\277\236", nullptr));
        label_show->setText(QCoreApplication::translate("MainWindow", "TextLabel", nullptr));
        pushButton_record->setText(QCoreApplication::translate("MainWindow", "\350\256\260\345\275\225", nullptr));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
