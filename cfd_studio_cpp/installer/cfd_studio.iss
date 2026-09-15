; Inno Setup script for Venturi CFD (C++/Qt6).
;
; Packages the already-standalone windeployqt output (build\app\) -- see
; cmake/DeployQt.cmake -- so this only zips up what's already there rather
; than duplicating dependency logic. Per-user install (PrivilegesRequired
; = lowest): no admin/UAC prompt, installs under %LOCALAPPDATA%, matching
; how a lot of modern lightweight Windows apps install. Shortcuts point
; straight at cfd_studio.exe (a GUI-subsystem exe, no console window),
; not at launch_cfd_studio.bat -- that .bat is a dev-only convenience
; that pre-dates windeployqt making the PATH prepend unnecessary, and
; launching a .bat from a shortcut briefly flashes a console window.
;
; Build with: ISCC.exe installer\cfd_studio.iss
; (after a normal .\build.ps1 -BuildGui build.)

#define AppSourceDir "..\build\app"
#define AppIcon "..\app\resources\app_icon.ico"

[Setup]
AppId={{167606BB-4C5C-4744-BC03-903CD72C3A63}
AppName=Venturi CFD
AppVersion=1.1.0
AppPublisher=Venturi CFD
DefaultDirName={localappdata}\Venturi CFD
DisableProgramGroupPage=yes
PrivilegesRequired=lowest
ArchitecturesAllowed=x64compatible
ArchitecturesInstallIn64BitMode=x64compatible
OutputDir=output
OutputBaseFilename=VenturiCFDSetup
SetupIconFile={#AppIcon}
UninstallDisplayIcon={app}\cfd_studio.exe
Compression=lzma2
SolidCompression=yes
WizardStyle=modern

[Languages]
Name: "english"; MessagesFile: "compiler:Default.isl"

[Tasks]
Name: "desktopicon"; Description: "Create a &desktop shortcut"; GroupDescription: "Additional shortcuts:"

[Files]
Source: "{#AppSourceDir}\*"; DestDir: "{app}"; Flags: ignoreversion recursesubdirs createallsubdirs; Excludes: "Makefile,cmake_install.cmake,CMakeFiles,cfd_studio_autogen,runs"

[Icons]
Name: "{userprograms}\Venturi CFD"; Filename: "{app}\cfd_studio.exe"
Name: "{userdesktop}\Venturi CFD"; Filename: "{app}\cfd_studio.exe"; Tasks: desktopicon

[Run]
Filename: "{app}\cfd_studio.exe"; Description: "Launch Venturi CFD"; Flags: nowait postinstall skipifsilent
