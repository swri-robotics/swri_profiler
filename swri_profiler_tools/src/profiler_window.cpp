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
#include <swri_profiler_tools/profiler_window.h>
#include <swri_profiler_tools/profile_database.h>

namespace swri_profiler_tools
{
ProfilerWindow::ProfilerWindow(ProfileDatabase *db)
  :
  QMainWindow(),
  db_(db)
{
  ui.setupUi(this);
  
  QObject::connect(ui.action_NewWindow, SIGNAL(triggered(bool)),
                   this, SIGNAL(createNewWindow()));

  connection_status_ = new QLabel("Not connected");
  statusBar()->addPermanentWidget(connection_status_);

  ui.profileTree->setDatabase(db_);
  ui.partitionWidget->setDatabase(db_);
  ui.timePlot->setDatabase(db_);

  QObject::connect(ui.profileTree, SIGNAL(activeNodeChanged(int,int)),
                   ui.partitionWidget, SLOT(setActiveNode(int,int)));
  QObject::connect(ui.partitionWidget, SIGNAL(activeNodeChanged(int,int)),
                   ui.profileTree, SLOT(setActiveNode(int,int)));
}

ProfilerWindow::~ProfilerWindow()
{
}

void ProfilerWindow::closeEvent(QCloseEvent *event)
{
  QMainWindow::closeEvent(event);
}

void ProfilerWindow::rosConnected(bool connected, QString master_uri)
{
  if (connected) {    
    statusBar()->showMessage("Connected to ROS Master " + master_uri);
    connection_status_->setText(master_uri);
  } else {
    statusBar()->showMessage("Disconnected from ROS Master");
    connection_status_->setText("Not connected");
  }
}
}  // namespace swri_profiler_tools
