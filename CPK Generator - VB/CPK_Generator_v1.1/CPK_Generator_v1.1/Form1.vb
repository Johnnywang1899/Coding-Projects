Imports System.ComponentModel
Imports Microsoft.WindowsAPICodePack.Taskbar
Imports System.IO
Imports Excel = Microsoft.Office.Interop.Excel 'import ms excel library

Public Class Form1
    Dim file As String 'location of input excel file
    Dim outputpath As String 'location of ouput cpk code file
    Dim x_index As Integer
    Dim y_index As Integer
    Dim original_y_index As Integer

    Private Sub Browse_Click(sender As Object, e As EventArgs) Handles Browse.Click
        Dim BrowseFile As New OpenFileDialog
        BrowseFile.Title = "Open a Excel File" 'define title of the file browse window
        BrowseFile.Filter = "Excel Files(*.xlsx)|*.xlsx*|All Files(*.*)|*.*" 'user can choose to only browse excel files or all files
        BrowseFile.ShowDialog() 'show file browse dialogue
        file = BrowseFile.FileName 'store the input file path selected by user
        'MessageBox.Show(file)
        TextBox2.Text = file 'fill the file path into the textbox
    End Sub

    Private Sub Exit_button_Click(sender As Object, e As EventArgs) Handles Exit_button.Click
        Close() 'close the program by cliking Exit
    End Sub

    Private Sub Save_To_Click(sender As Object, e As EventArgs) Handles Save_To.Click

        Dim FolderBrowserDialog1 As New FolderBrowserDialog
        'FolderBrowserDialog1.RootFolder = Environment.SpecialFolder.MyComputer 'define the folder browser's root folder
        FolderBrowserDialog1.SelectedPath = "C:\Users\yining.wang\Desktop\Test" 'define the folder browser's initial directory
        FolderBrowserDialog1.Description = "Save To..." 'add description onto file browser window
        FolderBrowserDialog1.ShowDialog() 'show file browser dialogue
        outputpath = FolderBrowserDialog1.SelectedPath 'store the output file path
        TextBox1.Text = outputpath 'fill the outputfile path into the textbox

    End Sub

    Private Sub Generate_Click(sender As Object, e As EventArgs) Handles Generate.Click

        If TextBox2.Text = "" Then
            MessageBox.Show("Please select a final tester output file.", "Warning", MessageBoxButtons.OK, MessageBoxIcon.Warning)
            'Status1.Visible = False
            Exit Sub
        End If


        If TextBox1.Text = "" Then
            MessageBox.Show("Please select a destination where you want the Minitab file containing CPK plot to be generated to.", "Warning", MessageBoxButtons.OK, MessageBoxIcon.Warning)
            'Status1.Visible = False
            Exit Sub
        End If

        If Row_index.Text = "" Then
            MessageBox.Show("Please fill in the row index of the first cell.", "Warning", MessageBoxButtons.OK, MessageBoxIcon.Warning)
        End If

        If Col_index.Text = "" Then
            MessageBox.Show("Please fill in the column index of the first cell.", "Warning", MessageBoxButtons.OK, MessageBoxIcon.Warning)
        End If

        'Generate formatted excel file
        Dim lastcolumn As Long
        Dim lastrow As Long
        Dim xlApp As Excel.Application
        Dim activeWorkBook As Excel.Workbook
        Dim activeWorkSheet As Excel.Worksheet
        Dim x_index_temp As String
        Dim y_index_temp As String
        Dim cell_name As String
        xlApp = New Excel.Application
        activeWorkBook = xlApp.Workbooks.Open(file)
        activeWorkSheet = activeWorkBook.Worksheets(1)
        lastcolumn = activeWorkSheet.UsedRange.SpecialCells(Excel.XlCellType.xlCellTypeLastCell).Column
        lastrow = activeWorkSheet.UsedRange.SpecialCells(Excel.XlCellType.xlCellTypeLastCell).Row
        Dim xlApp2 As Excel.Application
        Dim xlWorkBook As Excel.Workbook
        Dim xlWorkSheet As Excel.Worksheet
        Dim misValue As Object = System.Reflection.Missing.Value
        xlApp2 = New Excel.Application
        xlWorkBook = xlApp2.Workbooks.Add(misValue)
        xlWorkSheet = xlWorkBook.Sheets(1)

        x_index_temp = Convert.ToChar(Row_index.Text)
        y_index_temp = Convert.ToChar(Col_index.Text)
        cell_name = y_index_temp + x_index_temp
        x_index = activeWorkSheet.Range(cell_name).Row
        y_index = activeWorkSheet.Range(cell_name).Column
        original_y_index = y_index

        Dim x As Integer
        Dim y As Integer
        Dim row As Integer
        Dim column As Integer
        row = 2
        column = 1
        For x = x_index To lastrow
            'For y = 12 To lastcolumn
            For y = y_index To lastcolumn
                'y = 26
                xlWorkSheet.Cells(row, column) = activeWorkSheet.Cells(x, y)
                column = column + 2
            Next y
            column = 1
            row = row + 1
        Next x
        xlWorkSheet.SaveAs(outputpath & "\converted_data.xlsx")
        xlWorkBook.Close()
        xlApp2.Quit()
        '________________________________________________________________________________________________________________________________________________________________________________________________________________

        '_________________________________________________________________________________________________________________________________________________________________________________________________________
        'Generate minitab code
        Dim i As Integer = 1
        Dim j As Integer
        Dim k As Integer = 1

        Using cpkcode As StreamWriter = New StreamWriter(outputpath & "\CPK.mtb")
            ' Using ppkcode As StreamWriter = New StreamWriter(outputpath & "\PPK.mtb")
            cpkcode.Write(vbNewLine & "# =====Author = Yining (Johnny) Wang===== #")
            For j = y_index To lastcolumn Step 1
                'j = 26
                'ppkcode.Write(vbNewLine & " ")
                cpkcode.Write(vbNewLine & "Name c" & (i + 1) & " ""cpk" & k & """")
                cpkcode.Write(vbNewLine & "Capa C" & i)
                cpkcode.Write(" 1;")
                cpkcode.Write(vbNewLine & "  Lspec " & activeWorkSheet.Cells(2, j).Value & ";")
                cpkcode.Write(vbNewLine & "  Uspec " & activeWorkSheet.Cells(3, j).Value & ";")
                cpkcode.Write(vbNewLine & "  Pooled;")
                cpkcode.Write(vbNewLine & "  AMR;")
                'ppkcode.Write(vbNewLine & "CCRbar;")
                'ppkcode.Write(vbNewLine & "CCSbar;")
                'ppkcode.Write(vbNewLine & "CCAMR;")
                cpkcode.Write(vbNewLine & "  UnBiased;")
                cpkcode.Write(vbNewLine & "  OBiased;")
                'ppkcode.Write(vbNewLine & "Breakout 25;")
                cpkcode.Write(vbNewLine & "  Toler 6;")
                cpkcode.Write(vbNewLine & "  Within;")
                cpkcode.Write(vbNewLine & "  Overall;")
                cpkcode.Write(vbNewLine & "  CStat;")
                'ppkcode.Write(vbNewLine & "Test 1 2 5 6.")
                cpkcode.Write(vbNewLine & "CPK " & "'Cpk" & k & "'.")
                i = i + 2
                k = k + 1
            Next j
        End Using

        '________________________________________________________________________________________________________________________________________________________________________________________________________________
        'Generate minitab code for worksheet collection
        Using cpkcode As StreamWriter = New StreamWriter(outputpath & "\Exportedworksheet.mtb")

            cpkcode.Write(vbNewLine & "WSave """ & outputpath & "\Exportedworksheet.xls"";")
            cpkcode.Write(vbNewLine & "  FType;")
            cpkcode.Write(vbNewLine & "    Excel 97;")
            cpkcode.Write(vbNewLine & "  Missing;")
            cpkcode.Write(vbNewLine & "    Numeric '*' '*';")
            cpkcode.Write(vbNewLine & "    Text """" """";")
            cpkcode.Write(vbNewLine & "  Replace.")

        End Using

        'Generate minitab file
        Dim minitab As Mtb.Application
        minitab = New Mtb.Application
        Dim minitabproject As Mtb.Project
        'Dim minitabworksheet As Mtb.Worksheet
        minitabproject = minitab.ActiveProject


        'minitabworksheet = minitabproject.ActiveWorksheet
        With minitabproject
            .ExecuteCommand("WOpen '" & outputpath & "\converted_data.xlsx'; FType; Excel.")
            .ExecuteCommand("Execute '" & outputpath & "\CPK.mtb'")
            .ExecuteCommand("Execute '" & outputpath & "\Exportedworksheet.mtb'")
            .ExecuteCommand("Save '" & outputpath & "\CPK.MPJ'; Project; Replace.")
        End With

        '---------------------------------------------------------------------------------------------------------------------------------------------------------
        'Open exportedworksheet excel file
        '---------------------------------------------------------------------------------------------------------------------------------------------------------
        Dim xlapp_4 As New Excel.Application

        Dim xlworkbook_4 As Excel.Workbook
        Dim xlworksheet_4 As Excel.Worksheet

        xlworkbook_4 = xlapp_4.Workbooks.Open(outputpath & "\Exportedworksheet.xls")
        xlworksheet_4 = xlworkbook_4.Worksheets("Sheet1")

        Dim xlapp_5 As New Excel.Application

        Dim xlworkbook_5 As Excel.Workbook = xlapp_5.Workbooks.Add(misValue)
        Dim xlworksheet_5 As Excel.Worksheet = xlworkbook_5.Worksheets("Sheet1")

        Dim new_row As Integer = 2
        Dim new_col As Integer
        Dim count As Integer = 1
        Dim row_id As Integer = 1
        'Dim tem_storage As Long
        Dim new_lastcol As Integer = xlworksheet_4.UsedRange.SpecialCells(Excel.XlCellType.xlCellTypeLastCell).Column

        xlworksheet_5.Cells(1, 1) = "Index"
        xlworksheet_5.Cells(1, 2) = "Description"
        xlworksheet_5.Cells(1, 3) = "CPK"
        For new_col = 2 To new_lastcol Step 2
            ' If Not xlworksheet_5.Cells(2, new_col).value.Equals("") Then
            'xlworksheet_5.Cells(new_row, 1) = count
            xlworksheet_5.Cells(new_row, 3) = xlworksheet_4.Cells(2, new_col)
            xlworksheet_5.Cells(new_row, 2) = activeWorkSheet.Cells(1, original_y_index)
            xlworksheet_5.Cells(new_row, 1).value = "C" & row_id
            'MsgBox(xlworksheet_4.Cells(2, new_col))
            new_row = new_row + 1
            row_id = row_id + 2
            original_y_index = original_y_index + 1
            'count = count + 1
            'End If
        Next new_col

        xlworksheet_5.UsedRange.EntireColumn.AutoFit()
        xlworksheet_5.Range("A:C").HorizontalAlignment = Excel.XlVAlign.xlVAlignCenter
        xlworksheet_5.Range("A:C").VerticalAlignment = Excel.XlVAlign.xlVAlignCenter
        xlworksheet_5.Range("A:C").Font.Name = "Calibri"
        xlworksheet_5.Range("A:C").Font.Size = 12
        xlworksheet_5.Range("A1:C1").Interior.ColorIndex = 15
        xlworksheet_5.UsedRange.Borders.LineStyle = Excel.XlLineStyle.xlContinuous
        xlworksheet_5.UsedRange.Borders.Weight = Excel.XlBorderWeight.xlThin

        'chartsheet1.SaveAs(outputpath & "\PPK_Conclusion.xlsx")
        xlworkbook_5.SaveAs(outputpath & "\CPK_Conclusion.xlsx")


        'xlworksheet_5.SaveAs(outputpath & "\PPK_Conclusion.xlsx")
        ' chartsheet1.SaveAs(outputpath & "\PPK_Conclusion_diagram.xlsx")
        xlworkbook_4.Close()
        xlworkbook_5.Close()
        activeWorkBook.Close()
        xlapp_4.Quit()
        'xlapp_5.Quit()

        '--------------------------------------------------
        'ToolStripProgressBar1.Value = 100
        MessageBox.Show("Success!", "Notification", MessageBoxButtons.OK, MessageBoxIcon.Information)

    End Sub
End Class

