
#include <QDialog>
#include <QDateTime>
#include <QHBoxLayout>
#include <QMouseEvent>
#include <QVBoxLayout>

#include "selfdrive/ui/qt/widgets/groupWidget.h"



CGroupWidget::CGroupWidget( QString  title ) : QFrame(0) 
{
  m_bShow = 0;

  memset( m_pFrame, 0, sizeof(m_pFrame) );

  main_layout = new QVBoxLayout(this);
  main_layout->setMargin(0);


  hlayout = new QHBoxLayout;
  hlayout->setMargin(0);
  hlayout->setSpacing(20);

  // left icon 
  pix_plus =  QPixmap( "../assets/offroad/icon_plus.png" ).scaledToWidth(80, Qt::SmoothTransformation);
  pix_minus =  QPixmap( "../assets/offroad/icon_minus.png" ).scaledToWidth(80, Qt::SmoothTransformation);


  icon_label = new QLabel();
  icon_label->setPixmap(pix_plus );
  icon_label->setSizePolicy(QSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed));
  hlayout->addWidget(icon_label);



  // title
 // QString  title = "Steer control Method";
  title_label = new QPushButton(title);
  title_label->setFixedHeight(120);
  title_label->setStyleSheet("font-size: 50px; font-weight: 400; text-align: left");
  hlayout->addWidget(title_label);
  main_layout->addLayout(hlayout);


  connect(title_label, &QPushButton::clicked, [=]() {
    if( m_bShow )  m_bShow = 0;
    else   m_bShow = 1;
    refresh();
  });

/*
  // label
  method_label = new QPushButton("method"); // .setAlignment(Qt::AlignVCenter|Qt::AlignHCenter);
  method_label->setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #00A12E;
  )");
  method_label->setFixedSize( 500, 100);
  hlayout->addWidget(method_label);
  connect(method_label, &QPushButton::clicked, [=]() {
    m_nSelect += 1;
    if( m_nSelect > 1 )
      m_nSelect = 0;

    QString values = QString::number(m_nSelect);
    params.put("OpkrSteerMethod", values.toStdString());      
    refresh();
  });

  main_layout->addLayout(hlayout);

  FrameSmooth( parent );
  FrameNormal( parent );

  main_layout->addStretch();
  refresh();
*/
}

CGroupWidget::~CGroupWidget()
{
}


QVBoxLayout *CGroupWidget::CreateBoxLayout( int nID )
{
  if( nID < 0 ) nID = 0;
  else if( nID >= 10 ) return nullptr;
  
  if( m_pFrame[nID]) return nullptr;

  QFrame  *pFrame = new QFrame(); 
  pFrame->setContentsMargins(40, 10, 40, 50);
  pFrame->setStyleSheet(R"(
    * {
      padding: 0;
      border-radius: 50px;
      font-size: 35px;
      font-weight: 500;
      color: #E4E4E4;
      background-color: black;
    } 
  )");
  

  m_pFrame[nID] = pFrame;

  main_layout->addWidget(pFrame);
  QVBoxLayout *layout = new QVBoxLayout(pFrame);

  return  layout;
}


void CGroupWidget::showEvent(QShowEvent *event) 
{
  refresh();
}

void CGroupWidget::hideEvent(QHideEvent *event) 
{
  m_bShow = 0;
  refresh();
}

void CGroupWidget::refresh( int nID ) 
{

  if(  m_bShow == 0 )
  {
    // pmyWidget->setVisible(false);
    icon_label->setPixmap(pix_plus);
  }
  else
  {
    icon_label->setPixmap(pix_minus);
    //pmyWidget->setVisible(true);
  }  
 
 for( int i = 0; i<10; i++ )
 {
  if(  m_pFrame[i] == nullptr ) continue;
  
  if(  m_bShow == 0 )   m_pFrame[i]->hide();
  else 
  {
      if( nID == i ) m_pFrame[i]->show();
      else m_pFrame[i]->hide();
  }
 
 }

}




