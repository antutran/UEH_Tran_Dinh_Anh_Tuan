Set WshShell = CreateObject("WScript.Shell")
Set fso = CreateObject("Scripting.FileSystemObject")
currentDir = fso.GetParentFolderName(WScript.ScriptFullName)
batPath = currentDir & "\DUNG_ROBOT_VA_GAZEBO.bat"

' Tat hoan toan an khong hien terminal
WshShell.Run "cmd.exe /c """ & batPath & """", 0, False
