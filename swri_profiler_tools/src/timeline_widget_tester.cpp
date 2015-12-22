// *****************************************************************************
//
// Copyright (c) 2015, Southwest Research Institute® (SwRI®)
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of Southwest Research Institute® (SwRI®) nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL Southwest Research Institute® BE LIABLE 
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL 
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR 
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT 
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY 
// OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
// DAMAGE.
//
// *****************************************************************************
#include <swri_profiler_tools/timeline_widget_tester.h>

#include <QtGui>
#include <QApplication>
#include <QCoreApplication>

namespace swri_profiler_tools
{
TimelineWidgetTester::TimelineWidgetTester(QWidget *parent)
  :
  QDialog(parent)
{
  ui.setupUi(this);

  QObject::connect(ui.inMaximum, SIGNAL(valueChanged(int)),
                   this, SLOT(uiMaximumChanged(int)));  

  QObject::connect(ui.inResolution, SIGNAL(valueChanged(double)),
                   this, SLOT(uiResolutionChanged(double)));

  QObject::connect(ui.inCurrentIndex, SIGNAL(valueChanged(int)),
                   this, SLOT(uiCurrentChanged(int)));

  QObject::connect(ui.inHoverActive, SIGNAL(toggled(bool)),
                   this, SLOT(uiHoverChanged()));

  QObject::connect(ui.inHoverIndex, SIGNAL(valueChanged(int)),
                   this, SLOT(uiHoverChanged()));

  QObject::connect(ui.inRoiMin, SIGNAL(valueChanged(int)),
                   this, SLOT(uiRoiChanged()));

  QObject::connect(ui.inRoiMax, SIGNAL(valueChanged(int)),
                   this, SLOT(uiRoiChanged()));

  
  QObject::connect(ui.timelineWidget, SIGNAL(maximumChanged(size_t)),
                   this, SLOT(wiMaximumChanged(size_t)));

  QObject::connect(ui.timelineWidget, SIGNAL(resolutionChanged(double)),
                   this, SLOT(wiResolutionChanged(double)));

  QObject::connect(ui.timelineWidget, SIGNAL(currentChanged(size_t)),
                   this, SLOT(wiCurrentChanged(size_t)));

  QObject::connect(ui.timelineWidget, SIGNAL(hoverChanged(bool, size_t)),
                   this, SLOT(wiHoverChanged(bool, size_t)));

  QObject::connect(ui.timelineWidget, SIGNAL(roiChanged(size_t, size_t)),
                   this, SLOT(wiRoiChanged(size_t, size_t)));

  updateOutputs();
}

TimelineWidgetTester::~TimelineWidgetTester()
{
}

void TimelineWidgetTester::uiMaximumChanged(int max)
{
  ui.inCurrentIndex->setMaximum(2*max);
  ui.inHoverIndex->setMaximum(2*max);
  ui.inRoiMin->setMaximum(2*max);
  ui.inRoiMax->setMaximum(2*max);
  ui.timelineWidget->setMaximum(max);
}

void TimelineWidgetTester::wiMaximumChanged(size_t max)
{
  qWarning("widget maximum changed to %zu", max);
  updateOutputs();
}

void TimelineWidgetTester::uiResolutionChanged(double resolution)
{
  ui.timelineWidget->setResolution(resolution);
}

void TimelineWidgetTester::wiResolutionChanged(double resolution)
{
  qWarning("widget resolution changed to %lf", resolution);
  updateOutputs();
}

void TimelineWidgetTester::uiCurrentChanged(int idx)
{
  ui.timelineWidget->setCurrent(idx);
}

void TimelineWidgetTester::wiCurrentChanged(size_t idx)
{
  qWarning("widget current changed to %zu", idx);
  updateOutputs();
}

void TimelineWidgetTester::uiHoverChanged()
{
  ui.timelineWidget->setHover(ui.inHoverActive->isChecked(),
                              ui.inHoverIndex->value());
}

void TimelineWidgetTester::wiHoverChanged(bool active, size_t idx)
{
  qWarning("widget hover changed to %d, %zu", active, idx);
  updateOutputs();
}

void TimelineWidgetTester::uiRoiChanged()
{
  ui.timelineWidget->setRoi(ui.inRoiMin->value(), ui.inRoiMax->value());
}

void TimelineWidgetTester::wiRoiChanged(size_t min, size_t max)
{
  qWarning("widget roi changed to %zu, %zu", min, max);
  updateOutputs();
}

void TimelineWidgetTester::updateOutputs()
{
  ui.outMaximum->setText(QString("%1").arg(ui.timelineWidget->maximum()));
  ui.outResolution->setText(QString("%1").arg(ui.timelineWidget->resolution()));
  ui.outCurrentIndex->setText(QString("%1").arg(ui.timelineWidget->current()));
  ui.outHoverActive->setText(QString("%1").arg(ui.timelineWidget->hoverActive()));
  ui.outHoverIndex->setText(QString("%1").arg(ui.timelineWidget->hoverIndex()));
  ui.outRoiMin->setText(QString("%1").arg(ui.timelineWidget->roiMinimum()));
  ui.outRoiMax->setText(QString("%1").arg(ui.timelineWidget->roiMaximum()));
}
}  // namespace swri_profiler_tools


int main(int argc, char **argv)
{
  QApplication app(argc, argv);
  swri_profiler_tools::TimelineWidgetTester tester;
  tester.show();
  app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));
  return app.exec();
}


