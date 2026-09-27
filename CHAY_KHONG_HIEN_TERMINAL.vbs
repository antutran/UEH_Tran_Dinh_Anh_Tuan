Set WshShell = CreateObject("WScript.Shell")
Set fso = CreateObject("Scripting.FileSystemObject")
currentDir = fso.GetParentFolderName(WScript.ScriptFullName)
batPath = currentDir & "\CHAY_ROBOT_1_CLICK.bat"

' Chay file bat hoan toan an (0 = khong hien bat ky cua so terminal nao)
WshShell.Run "cmd.exe /c """ & batPath & """", 0, False
