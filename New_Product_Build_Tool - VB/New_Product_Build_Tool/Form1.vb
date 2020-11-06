Imports Excel = Microsoft.Office.Interop.Excel
Imports System
Public Class New_Product_Build_Tool

    Dim P_err_count As Integer
    Dim Q_err_count As Integer
    Dim i As Integer = 0
    Dim j As Integer
    Dim kit_truck_row As Integer = 1
    Dim kit_truck_col As Integer = 1
    Dim kit_truck_side As Integer = 1
    Dim file As String   'For part 1 & part 2
    Dim ori_file_path As String    'For part 2 "Inventory Check"
    Dim new_file_path As String    'For part 2 "Inventory Check"
    Dim new_side_button As Integer
    Dim textbox8_input_confirm As New Integer
    Dim xlapp_supply As New Excel.Application
    Dim supplyworkbook As Excel.Workbook
    Dim supplyworksheet As Excel.Worksheet
    Dim already_clicked As Integer = 0

    Private Sub Part_num_txtbox_TextChanged(sender As Object, e As EventArgs) Handles Part_num_txtbox.TextChanged
        If Not (Part_num_txtbox.Text.StartsWith("P") Or Part_num_txtbox.Text.Equals("")) Then
            P_err_count = P_err_count + 1
            If P_err_count < 3 Then
                MessageBox.Show("Invalid part number. Please scan again.", "Error", MessageBoxButtons.OK, MessageBoxIcon.Error)
                Part_num_txtbox.Clear()
            Else
                MessageBox.Show("Please cover other barcodes and then scan the part number.", "Warning", MessageBoxButtons.OK, MessageBoxIcon.Information)
                Part_num_txtbox.Clear()
                P_err_count = 0
            End If
        End If
        If Part_num_txtbox.Text.Length = 11 Then
            'SendKeys.Send("{Tab}")
            Ori_qty_txtbox.Select()
        End If
    End Sub

    Private Sub Ori_qty_txtbox_TextChanged(sender As Object, e As EventArgs) Handles Ori_qty_txtbox.TextChanged
        Timer1.Start()
        If Ori_qty_txtbox.Text = "" Then
            Timer1.Stop()
        End If
        If Not (Ori_qty_txtbox.Text.StartsWith("Q") Or Ori_qty_txtbox.Text = "") Then
            Q_err_count = Q_err_count + 1
            If Q_err_count < 3 Then
                MessageBox.Show("Invalid quantity. Please scan again.", "Error", MessageBoxButtons.OK, MessageBoxIcon.Error)
                Ori_qty_txtbox.Clear()
            Else
                MessageBox.Show("Please cover other barcodes and then scan the quantity.", "Warning", MessageBoxButtons.OK, MessageBoxIcon.Information)
                Ori_qty_txtbox.Clear()
                Q_err_count = 0
            End If
        End If
    End Sub

    Private Sub Add_btn_Click(sender As Object, e As EventArgs) Handles Add_btn.Click
        If (Part_num_txtbox.Text.Equals("") Or Ori_qty_txtbox.Text.Equals("")) Then
            MessageBox.Show("Please fill the space before adding!", "Warning", MessageBoxButtons.OK, MessageBoxIcon.Warning)
        Else
            Inventory_list.Rows.Add(Part_num_txtbox.Text, Ori_qty_txtbox.Text, " Side " & kit_truck_side & " Row " & kit_truck_row & " Column " & kit_truck_col & " ", Actual_qty_txtbox.Text)
            'xlworksheet_1.Cells(row_1, 1) = TextBox1.Text
            'xlworksheet_1.Cells(row_1, 2) = TextBox2.Text
            i = i + 1
            kit_truck_col = kit_truck_col + 1
            'row_1 = row_1 + 1
            Part_num_txtbox.Clear()
            Ori_qty_txtbox.Clear()
            Actual_qty_txtbox.Clear()
            P_err_count = 0
            Q_err_count = 0
            SendKeys.Send("{Tab}")
        End If
    End Sub

    Private Sub New_Product_Build_Tool_Load(sender As Object, e As EventArgs) Handles MyBase.Load
        Inventory_list.SelectionMode = DataGridViewSelectionMode.FullRowSelect
        Me.Label4.Visible = False
        Me.Label8.Visible = False
        Me.Label9.Visible = False
        Me.Label10.Visible = False
        PictureBox2.Show()
        P_err_count = 0
        Q_err_count = 0
    End Sub

    Private Sub Inventory_list_CellContentClick(sender As Object, e As DataGridViewCellEventArgs) Handles Inventory_list.CellContentClick
        Location_txtbox.Clear()
        Me.Location_txtbox.Enabled = True
        Me.Location_txtbox.TextAlign = HorizontalAlignment.Left
        Dim part_number As String
        Dim qty As String
        Dim loc As String
        Dim actual_qty As String
        part_number = Inventory_list.SelectedRows(0).Cells(0).Value
        qty = Inventory_list.SelectedRows(0).Cells(1).Value
        loc = Inventory_list.SelectedRows(0).Cells(2).Value
        actual_qty = Inventory_list.SelectedRows(0).Cells(3).Value
        Part_num_txtbox.Text = part_number
        Ori_qty_txtbox.Text = qty
        Location_txtbox.Text = loc
        Actual_qty_txtbox.Text = actual_qty
    End Sub

    Private Sub Update_btn_Click(sender As Object, e As EventArgs) Handles Update_btn.Click
        If (Part_num_txtbox.Text.Equals("") Or Ori_qty_txtbox.Text.Equals("") Or Location_txtbox.Text.Equals("")) Then
            MessageBox.Show("Please fill the space before updating!", "Warning", MessageBoxButtons.OK, MessageBoxIcon.Warning)
        Else
            Inventory_list.SelectedRows(0).Cells(0).Value = Part_num_txtbox.Text
            Inventory_list.SelectedRows(0).Cells(1).Value = Ori_qty_txtbox.Text
            Inventory_list.SelectedRows(0).Cells(2).Value = Location_txtbox.Text
            Inventory_list.SelectedRows(0).Cells(3).Value = Actual_qty_txtbox.Text
        End If
    End Sub

    Private Sub Delete_btn_Click(sender As Object, e As EventArgs) Handles Delete_btn.Click
        If MessageBox.Show("Are you sure to delete this row?", "DELETE", MessageBoxButtons.OKCancel, MessageBoxIcon.Question) = Windows.Forms.DialogResult.OK Then
            Inventory_list.Rows.RemoveAt(Inventory_list.SelectedRows(0).Index)
            i = i - 1
        End If
    End Sub

    Private Sub Save_btn_Click(sender As Object, e As EventArgs) Handles Save_btn.Click
        Dim xlapp_2 As New Excel.Application
        Dim xlworkbook_2 As Excel.Workbook
        Dim xlworksheet_2 As Excel.Worksheet
        Dim dialog As New FolderBrowserDialog()
        Dim misValue As Object = System.Reflection.Missing.Value
        Dim k As Integer
        Dim str_temp As String

        dialog.RootFolder = Environment.SpecialFolder.Desktop
        'dialog.SelectedPath = "C:\Users\yining.wang\Desktop\Kit Truck Scanning"
        dialog.Description = "Select the file path to save the document"
        dialog.ShowDialog()
        file = dialog.SelectedPath
        xlworkbook_2 = xlapp_2.Workbooks.Add(misValue)
        xlworksheet_2 = xlworkbook_2.Worksheets(1)
        xlworksheet_2.Cells(1, 1) = " Part number "
        xlworksheet_2.Cells(1, 2) = " Original Qty "
        xlworksheet_2.Cells(1, 3) = " Location "
        xlworksheet_2.Cells(1, 4) = " Actual Qty "
        xlworksheet_2.Cells(1, 5) = " Status "
        ' DataGridView1.Rows(1).Selected
        For j = 0 To (i - 1) Step 1
            For k = 0 To 3 Step 1
                'xlworksheet_2.Cells(j + 2, k + 1) = DataGridView1.Rows(j).Cells(k).Value
                If k < 2 Then
                    str_temp = Inventory_list.Rows(j).Cells(k).Value
                    xlworksheet_2.Cells(j + 2, k + 1) = Mid(str_temp, 2)
                Else
                    xlworksheet_2.Cells(j + 2, k + 1) = Inventory_list.Rows(j).Cells(k).Value
                End If
                'xlapp_2.WorksheetFunction.fit
            Next
        Next
        xlworksheet_2.UsedRange.EntireColumn.AutoFit()
        xlworksheet_2.Range("A:E").HorizontalAlignment = Excel.XlVAlign.xlVAlignCenter
        xlworksheet_2.Range("A:E").VerticalAlignment = Excel.XlVAlign.xlVAlignCenter
        xlworksheet_2.Range("A:E").Font.Name = "Calibri"
        xlworksheet_2.Range("A:E").Font.Size = 12
        xlworksheet_2.Range("A1:E1").Interior.ColorIndex = 15
        xlworksheet_2.UsedRange.Borders.LineStyle = Excel.XlLineStyle.xlContinuous
        xlworksheet_2.UsedRange.Borders.Weight = Excel.XlBorderWeight.xlThin
        xlworksheet_2.SaveAs(file & "\Kit Truck Scanning Result(Check).xlsx")
        xlworksheet_2.SaveAs(file & "\Kit Truck Scanning Result(Final).xlsx")
        'xlworkbook_1.Close()
        xlworkbook_2.Close()
        'xlapp_1.Quit()
        xlapp_2.Quit()
        If My.Computer.FileSystem.FileExists(file & "\Kit Truck Scanning Result(Final).xlsx") Then
            MessageBox.Show("You have successfully saved the file!", "Success", MessageBoxButtons.OKCancel, MessageBoxIcon.Information)
            ' Close()
        End If
    End Sub

    Private Sub New_row_btn_Click(sender As Object, e As EventArgs) Handles New_row_btn.Click
        kit_truck_row = kit_truck_row + 1
        kit_truck_col = 1
        Lock_chkbox.Checked = True
        If kit_truck_row > 3 Then
            MessageBox.Show("Please click ""New Side"" switch to the other side of the kit truck", "Switch Side", MessageBoxButtons.OK, MessageBoxIcon.Warning)
            kit_truck_row = kit_truck_row - 1
        End If
    End Sub

    Private Sub New_side_btn_Click(sender As Object, e As EventArgs) Handles New_side_btn.Click
        kit_truck_row = 1
        kit_truck_col = 1
        kit_truck_side = 2
        Lock_chkbox.Checked = True
        new_side_button = 1
        Me.New_side_btn.Enabled = False
    End Sub

    Private Sub Clear_btn_Click(sender As Object, e As EventArgs) Handles Clear_btn.Click
        Part_num_txtbox.Clear()
        Ori_qty_txtbox.Clear()
        Location_txtbox.Clear()
        Actual_qty_txtbox.Clear()
        Me.Location_txtbox.Enabled = False
        Me.Location_txtbox.TextAlign = HorizontalAlignment.Center
        Location_txtbox.Text = "(Automatic)"
    End Sub

    Private Sub Lock_chkbox_CheckedChanged(sender As Object, e As EventArgs) Handles Lock_chkbox.CheckedChanged
        If Lock_chkbox.Checked = True Then
            Me.New_side_btn.Enabled = False
            Me.New_row_btn.Enabled = False
        Else
            Me.New_row_btn.Enabled = True
            If new_side_button = 1 Then
                Me.New_side_btn.Enabled = False
            Else
                Me.New_side_btn.Enabled = True
            End If
        End If
    End Sub

    Private Sub Ori_file_btn_Click(sender As Object, e As EventArgs) Handles Ori_file_btn.Click
        Dim dog_ori As New OpenFileDialog
        dog_ori.Filter = "Excel Files |*.xls|All Files|*.*"
        dog_ori.Title = "Choose the original file"
        dog_ori.ShowDialog()
        ori_file_path = dog_ori.FileName
        TextBox4.Text = ori_file_path
    End Sub

    Private Sub SaveTo_btn_Click(sender As Object, e As EventArgs) Handles SaveTo_btn.Click
        Dim dog_new As New FolderBrowserDialog()
        dog_new.Description = "Please select the path to save the new file"
        dog_new.ShowDialog()
        new_file_path = dog_new.SelectedPath
        TextBox5.Text = new_file_path
    End Sub

    Private Sub Start_btn_Click(sender As Object, e As EventArgs) Handles Start_btn.Click
        If TextBox4.Text = "" Then
            MessageBox.Show("Please fill in the original file address", "Missing Information", MessageBoxButtons.OK, MessageBoxIcon.Warning)
            Exit Sub
        ElseIf TextBox5.Text = "" Then
            MessageBox.Show("Please fill in the file path for Save To...", "Missing Information", MessageBoxButtons.OK, MessageBoxIcon.Warning)
            Exit Sub
        End If

        Me.Label4.Visible = True
        Me.Label8.Visible = True
        Me.Label9.Visible = True
        Me.Label10.Visible = True
        Dim xlapp_req As New Excel.Application
        Dim xlapp_scan_check As New Excel.Application
        Dim xlapp_scan_final As New Excel.Application
        Dim req_workbook As Excel.Workbook
        Dim scan_workbook_check As Excel.Workbook
        Dim scan_workbook_final As Excel.Workbook
        Dim req_worksheet As Excel.Worksheet
        Dim scan_worksheet_check As Excel.Worksheet
        Dim scan_worksheet_final As Excel.Worksheet
        Dim scan_row As New Integer
        Dim req_row As New Integer
        Dim last_row_scan As New Integer
        Dim last_row_req As New Integer
        'Dim check_state As VariantType

        req_workbook = xlapp_req.Workbooks.Open(ori_file_path)
        req_worksheet = req_workbook.Worksheets(1)
        req_worksheet.UsedRange.UnMerge()
        req_worksheet.UsedRange.NumberFormat = "@"
        req_worksheet.UsedRange.WrapText = False
        req_worksheet.Cells(7, 57) = "Status"
        req_worksheet.Cells(7, 59) = "Qty"
        req_worksheet.Cells(7, 61) = "Location"
        req_workbook.Save()

        scan_workbook_check = xlapp_scan_check.Workbooks.Open(file & "\Kit Truck Scanning Result(Check).xlsx")
        'scan_workbook_check = xlapp_scan_check.Workbooks.Open("C:\Users\yining.wang\Desktop\Kit Truck Scanning" & "\Kit Truck Scanning Result(Check).xlsx")
        scan_worksheet_check = scan_workbook_check.Worksheets(1)
        scan_workbook_final = xlapp_scan_final.Workbooks.Open(file & "\Kit Truck Scanning Result(Final).xlsx")
        'scan_workbook_final = xlapp_scan_final.Workbooks.Open("C:\Users\yining.wang\Desktop\Kit Truck Scanning" & "\Kit Truck Scanning Result(Final).xlsx")
        scan_worksheet_final = scan_workbook_final.Worksheets(1)
        last_row_scan = scan_worksheet_final.UsedRange.SpecialCells(Excel.XlCellType.xlCellTypeLastCell).Row
        ' MsgBox(last_row_scan)

        last_row_req = req_worksheet.UsedRange.SpecialCells(Excel.XlCellType.xlCellTypeLastCell).Row
        ' MsgBox(last_row_req)
        For req_row = 1 To last_row_req Step 1
            If Not req_worksheet.Cells(req_row, 13).value = "" Then
                For scan_row = 2 To last_row_scan Step 1
                    ' check_state = req_worksheet.Cells(req_row, 13)
                    If req_worksheet.Cells(req_row, 13).value = scan_worksheet_check.Cells(scan_row, 1).value Then
                        'MsgBox(scan_row)
                        'scan_worksheet_final.Range(scan_worksheet_final.Cells(scan_row, 5)).Interior.ColorIndex = 4
                        scan_worksheet_final.Cells(scan_row, 5).Interior.ColorIndex = 4
                        req_worksheet.Cells(req_row, 57).Interior.ColorIndex = 4
                        req_worksheet.Cells(req_row, 59).value = scan_worksheet_final.Cells(scan_row, 4).value
                        req_worksheet.Cells(req_row, 61).value = scan_worksheet_final.Cells(scan_row, 3).value
                        'scan_worksheet_check.Rows(scan_row).delete
                        scan_worksheet_check.Rows(scan_row).clear()
                        Exit For
                    Else
                        Continue For
                    End If
                Next
            Else
                Continue For
            End If
        Next
        req_worksheet.Range("BE:BI").EntireColumn.AutoFit()
        req_worksheet.UsedRange.NumberFormat = "@"
        req_workbook.Save()
        scan_workbook_check.Save()
        scan_workbook_final.Save()
        xlapp_req.Quit()
        xlapp_scan_check.Quit()
        xlapp_scan_final.Quit()
        ProgressBar1.Value = 100
        MsgBox("Success!")
    End Sub

    Private Sub Search_txtbox_Click(sender As Object, e As EventArgs) Handles Search_txtbox.Click
        If Not textbox8_input_confirm = 1 Then
            Search_txtbox.Clear()
            Search_txtbox.BackColor = TransparencyKey
            Search_txtbox.TextAlign = HorizontalAlignment.Left
        End If
        textbox8_input_confirm = 1
    End Sub

    Private Sub Search_btn_Click(sender As Object, e As EventArgs) Handles Search_btn.Click
        Dim grid_row As New Integer
        'Dim grid_cell As New Integer
        For grid_row = 0 To (i - 1) Step 1
            If Search_txtbox.Text = Inventory_list.Rows(grid_row).Cells(0).Value Then
                Inventory_list.CurrentCell = Inventory_list.Rows(grid_row).Cells(0)
                Exit For
            Else
                Continue For
            End If
        Next
        If grid_row = i Then
            If Not Search_txtbox.Text = Inventory_list.Rows(grid_row - 1).Cells(0).Value Then
                MessageBox.Show("Item not found in the current list!", "Not Found", MessageBoxButtons.OK, MessageBoxIcon.Information)
            End If
        End If
    End Sub

    Private Sub Take_out_btn_Click(sender As Object, e As EventArgs) Handles Take_out_btn.Click
        If MessageBox.Show("Are you sure to take out this reel?", "Take Out", MessageBoxButtons.OKCancel, MessageBoxIcon.Question) = Windows.Forms.DialogResult.OK Then
            Inventory_list.Rows.RemoveAt(Inventory_list.CurrentRow.Index)
            i = i - 1
            Search_txtbox.Clear()
        End If
    End Sub

    Private Sub Timer1_Tick(sender As Object, e As EventArgs) Handles Timer1.Tick
        SendKeys.Send("{Tab}")
        Timer1.Stop()
    End Sub
    Private Sub partnum_supply_txtbox_Click(sender As Object, e As EventArgs) Handles partnum_supply_txtbox.Click
        If already_clicked = 0 Then
            supplyworkbook = xlapp_supply.Workbooks.Open("C:\Users\yining.wang\Desktop\Kit Truck Scanning" & "\Kit Truck Scanning Result(Final).xlsx")
            'supplyworkbook = xlapp_supply.Workbooks.Open(file & "\Kit Truck Scanning Result(Final).xlsx")
            supplyworksheet = supplyworkbook.Worksheets(1)
        Else
            Exit Sub
        End If
        already_clicked = 1
    End Sub

    Private Sub Search_Takeout_btn_Click(sender As Object, e As EventArgs) Handles Search_Takeout_btn.Click
        Dim supply_row As Integer
        Dim row_index As Integer = 0
        Dim supply_last_row As New Integer
        Dim temp_partnum As String
        supply_last_row = supplyworksheet.UsedRange.SpecialCells(Excel.XlCellType.xlCellTypeLastCell).Row
        For supply_row = 1 To supply_last_row Step 1
            row_index = row_index + 1
            If supplyworksheet.Cells(supply_row, 1).value = partnum_supply_txtbox.Text Then
                temp_partnum = partnum_supply_txtbox.Text
                If MessageBox.Show("Reel " + temp_partnum + " is located at " + supplyworksheet.Cells(supply_row, 3).value + ". Do you want to take the reel out?", "Search Item", MessageBoxButtons.YesNo, MessageBoxIcon.Question) = Windows.Forms.DialogResult.Yes Then
                    supplyworksheet.Rows(supply_row).clear()
                    partnum_supply_txtbox.Clear()
                    Exit For
                Else
                    Exit For
                End If
            End If
        Next
    End Sub

    Private Sub CLose_program_Click(sender As Object, e As EventArgs) Handles CLose_program.Click
        supplyworkbook.Save()
        supplyworkbook.Close()
        already_clicked = 0
        Close()
    End Sub

    Private Sub Format_2_btn_Click(sender As Object, e As EventArgs) Handles Format_2_btn.Click
        If TextBox4.Text = "" Then
            MessageBox.Show("Please fill in the original file address", "Missing Information", MessageBoxButtons.OK, MessageBoxIcon.Warning)
            Exit Sub
        ElseIf TextBox5.Text = "" Then
            MessageBox.Show("Please fill in the file path for Save To...", "Missing Information", MessageBoxButtons.OK, MessageBoxIcon.Warning)
            Exit Sub
        End If

        Me.Label4.Visible = True
        Me.Label8.Visible = True
        Me.Label9.Visible = True
        Me.Label10.Visible = True
        Dim xlapp_req As New Excel.Application
        Dim xlapp_scan_check As New Excel.Application
        Dim xlapp_scan_final As New Excel.Application
        Dim req_workbook As Excel.Workbook
        Dim scan_workbook_check As Excel.Workbook
        Dim scan_workbook_final As Excel.Workbook
        Dim req_worksheet As Excel.Worksheet
        Dim scan_worksheet_check As Excel.Worksheet
        Dim scan_worksheet_final As Excel.Worksheet
        Dim scan_row As New Integer
        Dim req_row As New Integer
        Dim last_row_scan As New Integer
        Dim last_row_req As New Integer
        'Dim check_state As VariantType

        req_workbook = xlapp_req.Workbooks.Open(ori_file_path)
        req_worksheet = req_workbook.Worksheets(1)
        req_worksheet.UsedRange.UnMerge()
        req_worksheet.UsedRange.NumberFormat = "@"
        req_worksheet.UsedRange.WrapText = False
        req_worksheet.Cells(5, 10) = "Status"
        req_worksheet.Cells(5, 11) = "Qty"
        req_worksheet.Cells(5, 12) = "Location"
        req_workbook.Save()

        'scan_workbook_check = xlapp_scan_check.Workbooks.Open(file & "\Kit Truck Scanning Result(Check).xlsx")
        scan_workbook_check = xlapp_scan_check.Workbooks.Open("C:\Users\yining.wang\Desktop\2017.12.19(Kit Truck Scanning)" & "\Kit Truck Scanning Result(Check).xlsx")
        scan_worksheet_check = scan_workbook_check.Worksheets(1)
        'scan_workbook_final = xlapp_scan_final.Workbooks.Open(file & "\Kit Truck Scanning Result(Final).xlsx")
        scan_workbook_final = xlapp_scan_final.Workbooks.Open("C:\Users\yining.wang\Desktop\2017.12.19(Kit Truck Scanning)" & "\Kit Truck Scanning Result(Final).xlsx")
        scan_worksheet_final = scan_workbook_final.Worksheets(1)
        last_row_scan = scan_worksheet_final.UsedRange.SpecialCells(Excel.XlCellType.xlCellTypeLastCell).Row
        ' MsgBox(last_row_scan)

        last_row_req = req_worksheet.UsedRange.SpecialCells(Excel.XlCellType.xlCellTypeLastCell).Row
        ' MsgBox(last_row_req)
        For req_row = 1 To last_row_req Step 1
            If Not req_worksheet.Cells(req_row, 2).value = "" Then
                For scan_row = 2 To last_row_scan Step 1
                    ' check_state = req_worksheet.Cells(req_row, 13)   
                    If req_worksheet.Cells(req_row, 2).value = scan_worksheet_check.Cells(scan_row, 1).value Then
                        'MsgBox(scan_row)
                        'scan_worksheet_final.Range(scan_worksheet_final.Cells(scan_row, 5)).Interior.ColorIndex = 4
                        scan_worksheet_final.Cells(scan_row, 5).Interior.ColorIndex = 4
                        req_worksheet.Cells(req_row, 10).Interior.ColorIndex = 4
                        req_worksheet.Cells(req_row, 11).value = scan_worksheet_final.Cells(scan_row, 4).value
                        req_worksheet.Cells(req_row, 12).value = scan_worksheet_final.Cells(scan_row, 3).value
                        'scan_worksheet_check.Rows(scan_row).delete
                        scan_worksheet_check.Rows(scan_row).clear()
                        Exit For
                    Else
                        Continue For
                    End If
                Next
            Else
                Continue For
            End If
        Next
        req_worksheet.Range("J:M").EntireColumn.AutoFit()
        req_worksheet.UsedRange.NumberFormat = "@"
        req_workbook.Save()
        scan_workbook_check.Save()
        scan_workbook_final.Save()
        xlapp_req.Quit()
        xlapp_scan_check.Quit()
        xlapp_scan_final.Quit()
        ProgressBar1.Value = 100
        MsgBox("Success!")
    End Sub

End Class
