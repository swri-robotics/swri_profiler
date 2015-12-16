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

#include <QtGui>
#include <QApplication>
#include <QCoreApplication>
#include <QDirIterator>
#include <QStringList>

#include <swri_profiler_tools/profiler_master.h>

void loadFonts()
{
  QStringList font_files;

  QDirIterator it(":/fonts", QDirIterator::Subdirectories);
  while (it.hasNext()) {
    it.next();
    
    if (!it.fileInfo().isFile()) {
      continue;
    }
    
    if (it.filePath().endsWith(".otf")) {
      font_files.append(it.filePath());
    }      

    if (it.filePath().endsWith(".ttf")) {
      font_files.append(it.filePath());
    }      
  }

  for (int i = 0; i < font_files.size(); i++) {
    
    int id = QFontDatabase::addApplicationFont(font_files[i]);
    if (id == -1) {
      qWarning() << "Failed to load font: " << font_files[i];
    } else {
      // qDebug() << "Loaded fonts from " << font_files[i] << ":";
      
      // QStringList families = QFontDatabase::applicationFontFamilies(id);
      // for (int j = 0; j < families.size(); j++) {
      //   qDebug() << "   " << families[j];
      // }
    }
  }
}

namespace swri_profiler_tools
{
void registerMetaTypes();
}

int main(int argc, char **argv)
{
  QApplication app(argc, argv);
  loadFonts();
  swri_profiler_tools::registerMetaTypes();

  QCoreApplication::setOrganizationName("Southwest Research Institute");
  QCoreApplication::setOrganizationDomain("swri.org");
  QCoreApplication::setApplicationName("SwRI Profiler");

  swri_profiler_tools::ProfilerMaster master;
  master.createNewWindow();
  app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));
  int result = app.exec();
  return result;
}
