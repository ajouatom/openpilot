

#pragma once

#include <QFrame>
#include <QLabel>
#include <QPushButton>
#include <QStackedLayout>
#include <QTimer>
#include <QWidget>

#include "selfdrive/ui/qt/widgets/controls.h"
#include "selfdrive/ui/ui.h"



class CGroupWidget : public QFrame 
{
  Q_OBJECT

public:
  explicit CGroupWidget( QString  title );
  ~CGroupWidget();

private:
  void showEvent(QShowEvent *event) override;
  void hideEvent(QHideEvent *event) override;

public slots:  
  virtual void refresh( int nID = 0 );



 private:
  QLabel *icon_label;
  QPixmap  pix_plus;
  QPixmap  pix_minus;

protected:
  QFrame  *m_pFrame[10];


protected:
  QVBoxLayout *main_layout;
  QHBoxLayout *hlayout;
  QPushButton *title_label;

protected:  
  int m_bShow;

  QVBoxLayout *CreateBoxLayout( int nID = 0);


};


