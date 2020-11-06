<Global.Microsoft.VisualBasic.CompilerServices.DesignerGenerated()>
Partial Class New_Product_Build_Tool
    Inherits System.Windows.Forms.Form

    'Form overrides dispose to clean up the component list.
    <System.Diagnostics.DebuggerNonUserCode()>
    Protected Overrides Sub Dispose(ByVal disposing As Boolean)
        Try
            If disposing AndAlso components IsNot Nothing Then
                components.Dispose()
            End If
        Finally
            MyBase.Dispose(disposing)
        End Try
    End Sub

    'Required by the Windows Form Designer
    Private components As System.ComponentModel.IContainer

    'NOTE: The following procedure is required by the Windows Form Designer
    'It can be modified using the Windows Form Designer.  
    'Do not modify it using the code editor.
    <System.Diagnostics.DebuggerStepThrough()>
    Private Sub InitializeComponent()
        Me.components = New System.ComponentModel.Container()
        Dim DataGridViewCellStyle4 As System.Windows.Forms.DataGridViewCellStyle = New System.Windows.Forms.DataGridViewCellStyle()
        Dim DataGridViewCellStyle5 As System.Windows.Forms.DataGridViewCellStyle = New System.Windows.Forms.DataGridViewCellStyle()
        Dim DataGridViewCellStyle6 As System.Windows.Forms.DataGridViewCellStyle = New System.Windows.Forms.DataGridViewCellStyle()
        Dim resources As System.ComponentModel.ComponentResourceManager = New System.ComponentModel.ComponentResourceManager(GetType(New_Product_Build_Tool))
        Me.TabControl1 = New System.Windows.Forms.TabControl()
        Me.TabPage1 = New System.Windows.Forms.TabPage()
        Me.Take_out_btn = New System.Windows.Forms.Button()
        Me.Search_btn = New System.Windows.Forms.Button()
        Me.Search_txtbox = New System.Windows.Forms.TextBox()
        Me.Actual_qty_txtbox = New System.Windows.Forms.TextBox()
        Me.Label7 = New System.Windows.Forms.Label()
        Me.Save_btn = New System.Windows.Forms.Button()
        Me.Lock_chkbox = New System.Windows.Forms.CheckBox()
        Me.Clear_btn = New System.Windows.Forms.Button()
        Me.Location_txtbox = New System.Windows.Forms.TextBox()
        Me.Label3 = New System.Windows.Forms.Label()
        Me.New_row_btn = New System.Windows.Forms.Button()
        Me.New_side_btn = New System.Windows.Forms.Button()
        Me.Delete_btn = New System.Windows.Forms.Button()
        Me.Update_btn = New System.Windows.Forms.Button()
        Me.Add_btn = New System.Windows.Forms.Button()
        Me.Inventory_list = New System.Windows.Forms.DataGridView()
        Me.Column1 = New System.Windows.Forms.DataGridViewTextBoxColumn()
        Me.Column2 = New System.Windows.Forms.DataGridViewTextBoxColumn()
        Me.Column3 = New System.Windows.Forms.DataGridViewTextBoxColumn()
        Me.Column4 = New System.Windows.Forms.DataGridViewTextBoxColumn()
        Me.Ori_qty_txtbox = New System.Windows.Forms.TextBox()
        Me.Label2 = New System.Windows.Forms.Label()
        Me.Part_num_txtbox = New System.Windows.Forms.TextBox()
        Me.Label1 = New System.Windows.Forms.Label()
        Me.TabPage2 = New System.Windows.Forms.TabPage()
        Me.PictureBox2 = New System.Windows.Forms.PictureBox()
        Me.PictureBox1 = New System.Windows.Forms.PictureBox()
        Me.Format_2_btn = New System.Windows.Forms.Button()
        Me.Label10 = New System.Windows.Forms.Label()
        Me.Label9 = New System.Windows.Forms.Label()
        Me.Label8 = New System.Windows.Forms.Label()
        Me.Label4 = New System.Windows.Forms.Label()
        Me.ProgressBar1 = New System.Windows.Forms.ProgressBar()
        Me.Start_btn = New System.Windows.Forms.Button()
        Me.SaveTo_btn = New System.Windows.Forms.Button()
        Me.TextBox5 = New System.Windows.Forms.TextBox()
        Me.Label6 = New System.Windows.Forms.Label()
        Me.Ori_file_btn = New System.Windows.Forms.Button()
        Me.TextBox4 = New System.Windows.Forms.TextBox()
        Me.Label5 = New System.Windows.Forms.Label()
        Me.TabPage3 = New System.Windows.Forms.TabPage()
        Me.CLose_program = New System.Windows.Forms.Button()
        Me.Search_Takeout_btn = New System.Windows.Forms.Button()
        Me.partnum_supply_txtbox = New System.Windows.Forms.TextBox()
        Me.Label11 = New System.Windows.Forms.Label()
        Me.Timer1 = New System.Windows.Forms.Timer(Me.components)
        Me.TabControl1.SuspendLayout()
        Me.TabPage1.SuspendLayout()
        CType(Me.Inventory_list, System.ComponentModel.ISupportInitialize).BeginInit()
        Me.TabPage2.SuspendLayout()
        CType(Me.PictureBox2, System.ComponentModel.ISupportInitialize).BeginInit()
        CType(Me.PictureBox1, System.ComponentModel.ISupportInitialize).BeginInit()
        Me.TabPage3.SuspendLayout()
        Me.SuspendLayout()
        '
        'TabControl1
        '
        Me.TabControl1.Controls.Add(Me.TabPage1)
        Me.TabControl1.Controls.Add(Me.TabPage2)
        Me.TabControl1.Controls.Add(Me.TabPage3)
        Me.TabControl1.Location = New System.Drawing.Point(38, 27)
        Me.TabControl1.Name = "TabControl1"
        Me.TabControl1.SelectedIndex = 0
        Me.TabControl1.Size = New System.Drawing.Size(536, 740)
        Me.TabControl1.TabIndex = 0
        Me.TabControl1.TabStop = False
        Me.TabControl1.Tag = ""
        '
        'TabPage1
        '
        Me.TabPage1.BackColor = System.Drawing.Color.Transparent
        Me.TabPage1.Controls.Add(Me.Take_out_btn)
        Me.TabPage1.Controls.Add(Me.Search_btn)
        Me.TabPage1.Controls.Add(Me.Search_txtbox)
        Me.TabPage1.Controls.Add(Me.Actual_qty_txtbox)
        Me.TabPage1.Controls.Add(Me.Label7)
        Me.TabPage1.Controls.Add(Me.Save_btn)
        Me.TabPage1.Controls.Add(Me.Lock_chkbox)
        Me.TabPage1.Controls.Add(Me.Clear_btn)
        Me.TabPage1.Controls.Add(Me.Location_txtbox)
        Me.TabPage1.Controls.Add(Me.Label3)
        Me.TabPage1.Controls.Add(Me.New_row_btn)
        Me.TabPage1.Controls.Add(Me.New_side_btn)
        Me.TabPage1.Controls.Add(Me.Delete_btn)
        Me.TabPage1.Controls.Add(Me.Update_btn)
        Me.TabPage1.Controls.Add(Me.Add_btn)
        Me.TabPage1.Controls.Add(Me.Inventory_list)
        Me.TabPage1.Controls.Add(Me.Ori_qty_txtbox)
        Me.TabPage1.Controls.Add(Me.Label2)
        Me.TabPage1.Controls.Add(Me.Part_num_txtbox)
        Me.TabPage1.Controls.Add(Me.Label1)
        Me.TabPage1.Location = New System.Drawing.Point(4, 22)
        Me.TabPage1.Name = "TabPage1"
        Me.TabPage1.Padding = New System.Windows.Forms.Padding(3)
        Me.TabPage1.Size = New System.Drawing.Size(528, 714)
        Me.TabPage1.TabIndex = 0
        Me.TabPage1.Text = "Kit Truck Scan"
        '
        'Take_out_btn
        '
        Me.Take_out_btn.Location = New System.Drawing.Point(386, 660)
        Me.Take_out_btn.Name = "Take_out_btn"
        Me.Take_out_btn.Size = New System.Drawing.Size(83, 36)
        Me.Take_out_btn.TabIndex = 15
        Me.Take_out_btn.TabStop = False
        Me.Take_out_btn.Text = "Take Out"
        Me.Take_out_btn.UseVisualStyleBackColor = True
        '
        'Search_btn
        '
        Me.Search_btn.Location = New System.Drawing.Point(287, 660)
        Me.Search_btn.Name = "Search_btn"
        Me.Search_btn.Size = New System.Drawing.Size(83, 36)
        Me.Search_btn.TabIndex = 14
        Me.Search_btn.TabStop = False
        Me.Search_btn.Text = "Search"
        Me.Search_btn.UseVisualStyleBackColor = True
        '
        'Search_txtbox
        '
        Me.Search_txtbox.BackColor = System.Drawing.Color.Gainsboro
        Me.Search_txtbox.Font = New System.Drawing.Font("Calibri", 12.0!, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, CType(0, Byte))
        Me.Search_txtbox.Location = New System.Drawing.Point(16, 665)
        Me.Search_txtbox.Name = "Search_txtbox"
        Me.Search_txtbox.Size = New System.Drawing.Size(254, 27)
        Me.Search_txtbox.TabIndex = 13
        Me.Search_txtbox.TabStop = False
        Me.Search_txtbox.Text = "(Add ""P"" before the part number)"
        Me.Search_txtbox.TextAlign = System.Windows.Forms.HorizontalAlignment.Center
        '
        'Actual_qty_txtbox
        '
        Me.Actual_qty_txtbox.Font = New System.Drawing.Font("Calibri", 11.25!, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, CType(0, Byte))
        Me.Actual_qty_txtbox.Location = New System.Drawing.Point(332, 70)
        Me.Actual_qty_txtbox.Multiline = True
        Me.Actual_qty_txtbox.Name = "Actual_qty_txtbox"
        Me.Actual_qty_txtbox.Size = New System.Drawing.Size(76, 29)
        Me.Actual_qty_txtbox.TabIndex = 2
        '
        'Label7
        '
        Me.Label7.AutoSize = True
        Me.Label7.Font = New System.Drawing.Font("Calibri", 14.25!, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, CType(0, Byte))
        Me.Label7.Location = New System.Drawing.Point(227, 70)
        Me.Label7.Name = "Label7"
        Me.Label7.Size = New System.Drawing.Size(99, 23)
        Me.Label7.TabIndex = 12
        Me.Label7.Text = "Actual Qty:"
        '
        'Save_btn
        '
        Me.Save_btn.Location = New System.Drawing.Point(122, 572)
        Me.Save_btn.Name = "Save_btn"
        Me.Save_btn.Size = New System.Drawing.Size(248, 57)
        Me.Save_btn.TabIndex = 1
        Me.Save_btn.TabStop = False
        Me.Save_btn.Text = "Save"
        Me.Save_btn.UseVisualStyleBackColor = True
        '
        'Lock_chkbox
        '
        Me.Lock_chkbox.AutoSize = True
        Me.Lock_chkbox.Checked = True
        Me.Lock_chkbox.CheckState = System.Windows.Forms.CheckState.Checked
        Me.Lock_chkbox.Location = New System.Drawing.Point(64, 170)
        Me.Lock_chkbox.Name = "Lock_chkbox"
        Me.Lock_chkbox.Size = New System.Drawing.Size(50, 17)
        Me.Lock_chkbox.TabIndex = 11
        Me.Lock_chkbox.TabStop = False
        Me.Lock_chkbox.Text = "Lock"
        Me.Lock_chkbox.UseVisualStyleBackColor = True
        '
        'Clear_btn
        '
        Me.Clear_btn.Location = New System.Drawing.Point(423, 154)
        Me.Clear_btn.Name = "Clear_btn"
        Me.Clear_btn.Size = New System.Drawing.Size(69, 48)
        Me.Clear_btn.TabIndex = 10
        Me.Clear_btn.TabStop = False
        Me.Clear_btn.Text = "Clear Text " & Global.Microsoft.VisualBasic.ChrW(13) & Global.Microsoft.VisualBasic.ChrW(10) & "and " & Global.Microsoft.VisualBasic.ChrW(13) & Global.Microsoft.VisualBasic.ChrW(10) & "No Change"
        Me.Clear_btn.UseVisualStyleBackColor = True
        '
        'Location_txtbox
        '
        Me.Location_txtbox.Enabled = False
        Me.Location_txtbox.Font = New System.Drawing.Font("Calibri", 11.25!, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, CType(0, Byte))
        Me.Location_txtbox.Location = New System.Drawing.Point(122, 110)
        Me.Location_txtbox.Multiline = True
        Me.Location_txtbox.Name = "Location_txtbox"
        Me.Location_txtbox.Size = New System.Drawing.Size(286, 29)
        Me.Location_txtbox.TabIndex = 8
        Me.Location_txtbox.Text = "(Automatic)"
        Me.Location_txtbox.TextAlign = System.Windows.Forms.HorizontalAlignment.Center
        '
        'Label3
        '
        Me.Label3.AutoSize = True
        Me.Label3.Cursor = System.Windows.Forms.Cursors.WaitCursor
        Me.Label3.Font = New System.Drawing.Font("Calibri", 14.25!, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, CType(0, Byte))
        Me.Label3.Location = New System.Drawing.Point(6, 110)
        Me.Label3.Name = "Label3"
        Me.Label3.Size = New System.Drawing.Size(81, 23)
        Me.Label3.TabIndex = 7
        Me.Label3.Text = "Location:"
        '
        'New_row_btn
        '
        Me.New_row_btn.Location = New System.Drawing.Point(122, 153)
        Me.New_row_btn.Name = "New_row_btn"
        Me.New_row_btn.Size = New System.Drawing.Size(204, 49)
        Me.New_row_btn.TabIndex = 6
        Me.New_row_btn.TabStop = False
        Me.New_row_btn.Text = "New Row"
        Me.New_row_btn.UseVisualStyleBackColor = True
        '
        'New_side_btn
        '
        Me.New_side_btn.Location = New System.Drawing.Point(332, 153)
        Me.New_side_btn.Name = "New_side_btn"
        Me.New_side_btn.Size = New System.Drawing.Size(76, 49)
        Me.New_side_btn.TabIndex = 5
        Me.New_side_btn.TabStop = False
        Me.New_side_btn.Text = "New Side"
        Me.New_side_btn.UseVisualStyleBackColor = True
        '
        'Delete_btn
        '
        Me.Delete_btn.Location = New System.Drawing.Point(423, 110)
        Me.Delete_btn.Name = "Delete_btn"
        Me.Delete_btn.Size = New System.Drawing.Size(69, 29)
        Me.Delete_btn.TabIndex = 4
        Me.Delete_btn.TabStop = False
        Me.Delete_btn.Text = "Delete"
        Me.Delete_btn.UseVisualStyleBackColor = True
        '
        'Update_btn
        '
        Me.Update_btn.Location = New System.Drawing.Point(423, 71)
        Me.Update_btn.Name = "Update_btn"
        Me.Update_btn.Size = New System.Drawing.Size(69, 29)
        Me.Update_btn.TabIndex = 5
        Me.Update_btn.TabStop = False
        Me.Update_btn.Text = "Update"
        Me.Update_btn.UseVisualStyleBackColor = True
        '
        'Add_btn
        '
        Me.Add_btn.Location = New System.Drawing.Point(423, 30)
        Me.Add_btn.Name = "Add_btn"
        Me.Add_btn.Size = New System.Drawing.Size(69, 29)
        Me.Add_btn.TabIndex = 3
        Me.Add_btn.Text = "Add"
        Me.Add_btn.UseVisualStyleBackColor = True
        '
        'Inventory_list
        '
        Me.Inventory_list.AccessibleRole = System.Windows.Forms.AccessibleRole.None
        Me.Inventory_list.AllowUserToAddRows = False
        Me.Inventory_list.AllowUserToDeleteRows = False
        DataGridViewCellStyle4.Alignment = System.Windows.Forms.DataGridViewContentAlignment.MiddleLeft
        DataGridViewCellStyle4.BackColor = System.Drawing.SystemColors.Control
        DataGridViewCellStyle4.Font = New System.Drawing.Font("Microsoft Sans Serif", 8.25!, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, CType(0, Byte))
        DataGridViewCellStyle4.ForeColor = System.Drawing.SystemColors.WindowText
        DataGridViewCellStyle4.SelectionBackColor = System.Drawing.SystemColors.Highlight
        DataGridViewCellStyle4.SelectionForeColor = System.Drawing.SystemColors.HighlightText
        DataGridViewCellStyle4.WrapMode = System.Windows.Forms.DataGridViewTriState.[True]
        Me.Inventory_list.ColumnHeadersDefaultCellStyle = DataGridViewCellStyle4
        Me.Inventory_list.ColumnHeadersHeightSizeMode = System.Windows.Forms.DataGridViewColumnHeadersHeightSizeMode.AutoSize
        Me.Inventory_list.Columns.AddRange(New System.Windows.Forms.DataGridViewColumn() {Me.Column1, Me.Column2, Me.Column3, Me.Column4})
        DataGridViewCellStyle5.Alignment = System.Windows.Forms.DataGridViewContentAlignment.MiddleLeft
        DataGridViewCellStyle5.BackColor = System.Drawing.SystemColors.Window
        DataGridViewCellStyle5.Font = New System.Drawing.Font("Microsoft Sans Serif", 8.25!, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, CType(0, Byte))
        DataGridViewCellStyle5.ForeColor = System.Drawing.SystemColors.ControlText
        DataGridViewCellStyle5.SelectionBackColor = System.Drawing.SystemColors.Highlight
        DataGridViewCellStyle5.SelectionForeColor = System.Drawing.SystemColors.HighlightText
        DataGridViewCellStyle5.WrapMode = System.Windows.Forms.DataGridViewTriState.[False]
        Me.Inventory_list.DefaultCellStyle = DataGridViewCellStyle5
        Me.Inventory_list.Location = New System.Drawing.Point(16, 220)
        Me.Inventory_list.Name = "Inventory_list"
        Me.Inventory_list.ReadOnly = True
        DataGridViewCellStyle6.Alignment = System.Windows.Forms.DataGridViewContentAlignment.MiddleLeft
        DataGridViewCellStyle6.BackColor = System.Drawing.SystemColors.Control
        DataGridViewCellStyle6.Font = New System.Drawing.Font("Microsoft Sans Serif", 8.25!, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, CType(0, Byte))
        DataGridViewCellStyle6.ForeColor = System.Drawing.SystemColors.WindowText
        DataGridViewCellStyle6.SelectionBackColor = System.Drawing.SystemColors.Highlight
        DataGridViewCellStyle6.SelectionForeColor = System.Drawing.SystemColors.HighlightText
        DataGridViewCellStyle6.WrapMode = System.Windows.Forms.DataGridViewTriState.[True]
        Me.Inventory_list.RowHeadersDefaultCellStyle = DataGridViewCellStyle6
        Me.Inventory_list.Size = New System.Drawing.Size(476, 320)
        Me.Inventory_list.TabIndex = 0
        Me.Inventory_list.TabStop = False
        '
        'Column1
        '
        Me.Column1.FillWeight = 98.47718!
        Me.Column1.HeaderText = "Part Number"
        Me.Column1.Name = "Column1"
        Me.Column1.ReadOnly = True
        '
        'Column2
        '
        Me.Column2.FillWeight = 101.5229!
        Me.Column2.HeaderText = "Original Qty"
        Me.Column2.Name = "Column2"
        Me.Column2.ReadOnly = True
        Me.Column2.Width = 90
        '
        'Column3
        '
        Me.Column3.HeaderText = "Location"
        Me.Column3.Name = "Column3"
        Me.Column3.ReadOnly = True
        Me.Column3.Width = 150
        '
        'Column4
        '
        Me.Column4.AutoSizeMode = System.Windows.Forms.DataGridViewAutoSizeColumnMode.Fill
        Me.Column4.HeaderText = "Actual Qty"
        Me.Column4.Name = "Column4"
        Me.Column4.ReadOnly = True
        '
        'Ori_qty_txtbox
        '
        Me.Ori_qty_txtbox.Font = New System.Drawing.Font("Calibri", 11.25!, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, CType(0, Byte))
        Me.Ori_qty_txtbox.Location = New System.Drawing.Point(122, 71)
        Me.Ori_qty_txtbox.Multiline = True
        Me.Ori_qty_txtbox.Name = "Ori_qty_txtbox"
        Me.Ori_qty_txtbox.Size = New System.Drawing.Size(76, 29)
        Me.Ori_qty_txtbox.TabIndex = 1
        '
        'Label2
        '
        Me.Label2.AutoSize = True
        Me.Label2.Font = New System.Drawing.Font("Calibri", 14.25!, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, CType(0, Byte))
        Me.Label2.Location = New System.Drawing.Point(6, 71)
        Me.Label2.Name = "Label2"
        Me.Label2.Size = New System.Drawing.Size(111, 23)
        Me.Label2.TabIndex = 0
        Me.Label2.Text = "Original Qty:"
        '
        'Part_num_txtbox
        '
        Me.Part_num_txtbox.Font = New System.Drawing.Font("Calibri", 11.25!, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, CType(0, Byte))
        Me.Part_num_txtbox.Location = New System.Drawing.Point(122, 30)
        Me.Part_num_txtbox.Multiline = True
        Me.Part_num_txtbox.Name = "Part_num_txtbox"
        Me.Part_num_txtbox.Size = New System.Drawing.Size(286, 29)
        Me.Part_num_txtbox.TabIndex = 0
        '
        'Label1
        '
        Me.Label1.AutoSize = True
        Me.Label1.Font = New System.Drawing.Font("Calibri", 14.25!, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, CType(0, Byte))
        Me.Label1.Location = New System.Drawing.Point(7, 31)
        Me.Label1.Name = "Label1"
        Me.Label1.Size = New System.Drawing.Size(117, 23)
        Me.Label1.TabIndex = 0
        Me.Label1.Text = "Part Number:"
        '
        'TabPage2
        '
        Me.TabPage2.BackColor = System.Drawing.Color.WhiteSmoke
        Me.TabPage2.Controls.Add(Me.PictureBox2)
        Me.TabPage2.Controls.Add(Me.PictureBox1)
        Me.TabPage2.Controls.Add(Me.Format_2_btn)
        Me.TabPage2.Controls.Add(Me.Label10)
        Me.TabPage2.Controls.Add(Me.Label9)
        Me.TabPage2.Controls.Add(Me.Label8)
        Me.TabPage2.Controls.Add(Me.Label4)
        Me.TabPage2.Controls.Add(Me.ProgressBar1)
        Me.TabPage2.Controls.Add(Me.Start_btn)
        Me.TabPage2.Controls.Add(Me.SaveTo_btn)
        Me.TabPage2.Controls.Add(Me.TextBox5)
        Me.TabPage2.Controls.Add(Me.Label6)
        Me.TabPage2.Controls.Add(Me.Ori_file_btn)
        Me.TabPage2.Controls.Add(Me.TextBox4)
        Me.TabPage2.Controls.Add(Me.Label5)
        Me.TabPage2.Font = New System.Drawing.Font("Calibri", 9.75!, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, CType(0, Byte))
        Me.TabPage2.Location = New System.Drawing.Point(4, 22)
        Me.TabPage2.Name = "TabPage2"
        Me.TabPage2.Padding = New System.Windows.Forms.Padding(3)
        Me.TabPage2.Size = New System.Drawing.Size(528, 714)
        Me.TabPage2.TabIndex = 1
        Me.TabPage2.Text = "Inventory Check"
        '
        'PictureBox2
        '
        Me.PictureBox2.Cursor = System.Windows.Forms.Cursors.Arrow
        Me.PictureBox2.Image = CType(resources.GetObject("PictureBox2.Image"), System.Drawing.Image)
        Me.PictureBox2.Location = New System.Drawing.Point(281, 271)
        Me.PictureBox2.Margin = New System.Windows.Forms.Padding(0)
        Me.PictureBox2.Name = "PictureBox2"
        Me.PictureBox2.Size = New System.Drawing.Size(230, 248)
        Me.PictureBox2.TabIndex = 14
        Me.PictureBox2.TabStop = False
        '
        'PictureBox1
        '
        Me.PictureBox1.Location = New System.Drawing.Point(25, 271)
        Me.PictureBox1.Name = "PictureBox1"
        Me.PictureBox1.Size = New System.Drawing.Size(230, 248)
        Me.PictureBox1.TabIndex = 13
        Me.PictureBox1.TabStop = False
        '
        'Format_2_btn
        '
        Me.Format_2_btn.Location = New System.Drawing.Point(349, 548)
        Me.Format_2_btn.Name = "Format_2_btn"
        Me.Format_2_btn.Size = New System.Drawing.Size(107, 57)
        Me.Format_2_btn.TabIndex = 12
        Me.Format_2_btn.Text = "Start" & Global.Microsoft.VisualBasic.ChrW(13) & Global.Microsoft.VisualBasic.ChrW(10) & "(format 2)"
        Me.Format_2_btn.UseVisualStyleBackColor = True
        '
        'Label10
        '
        Me.Label10.AutoSize = True
        Me.Label10.Font = New System.Drawing.Font("Calibri", 15.75!, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, CType(0, Byte))
        Me.Label10.Location = New System.Drawing.Point(164, 624)
        Me.Label10.Name = "Label10"
        Me.Label10.Size = New System.Drawing.Size(18, 26)
        Me.Label10.TabIndex = 11
        Me.Label10.Text = "."
        '
        'Label9
        '
        Me.Label9.AutoSize = True
        Me.Label9.Font = New System.Drawing.Font("Calibri", 15.75!, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, CType(0, Byte))
        Me.Label9.Location = New System.Drawing.Point(141, 624)
        Me.Label9.Name = "Label9"
        Me.Label9.Size = New System.Drawing.Size(18, 26)
        Me.Label9.TabIndex = 10
        Me.Label9.Text = "."
        '
        'Label8
        '
        Me.Label8.AutoSize = True
        Me.Label8.Font = New System.Drawing.Font("Calibri", 15.75!, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, CType(0, Byte))
        Me.Label8.Location = New System.Drawing.Point(117, 624)
        Me.Label8.Name = "Label8"
        Me.Label8.Size = New System.Drawing.Size(18, 26)
        Me.Label8.TabIndex = 9
        Me.Label8.Text = "."
        '
        'Label4
        '
        Me.Label4.AutoSize = True
        Me.Label4.Font = New System.Drawing.Font("Calibri", 15.75!, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, CType(0, Byte))
        Me.Label4.Location = New System.Drawing.Point(6, 624)
        Me.Label4.Name = "Label4"
        Me.Label4.Size = New System.Drawing.Size(103, 26)
        Me.Label4.TabIndex = 8
        Me.Label4.Text = "Processing"
        '
        'ProgressBar1
        '
        Me.ProgressBar1.Location = New System.Drawing.Point(6, 662)
        Me.ProgressBar1.Name = "ProgressBar1"
        Me.ProgressBar1.Size = New System.Drawing.Size(491, 23)
        Me.ProgressBar1.TabIndex = 7
        '
        'Start_btn
        '
        Me.Start_btn.Location = New System.Drawing.Point(84, 548)
        Me.Start_btn.Name = "Start_btn"
        Me.Start_btn.Size = New System.Drawing.Size(107, 57)
        Me.Start_btn.TabIndex = 6
        Me.Start_btn.Text = "Start" & Global.Microsoft.VisualBasic.ChrW(13) & Global.Microsoft.VisualBasic.ChrW(10) & "(format 1)"
        Me.Start_btn.UseVisualStyleBackColor = True
        '
        'SaveTo_btn
        '
        Me.SaveTo_btn.Location = New System.Drawing.Point(475, 207)
        Me.SaveTo_btn.Name = "SaveTo_btn"
        Me.SaveTo_btn.Size = New System.Drawing.Size(36, 23)
        Me.SaveTo_btn.TabIndex = 5
        Me.SaveTo_btn.Text = "..."
        Me.SaveTo_btn.UseVisualStyleBackColor = True
        '
        'TextBox5
        '
        Me.TextBox5.Location = New System.Drawing.Point(25, 207)
        Me.TextBox5.Name = "TextBox5"
        Me.TextBox5.Size = New System.Drawing.Size(404, 23)
        Me.TextBox5.TabIndex = 4
        '
        'Label6
        '
        Me.Label6.AutoSize = True
        Me.Label6.Font = New System.Drawing.Font("Calibri", 14.25!, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, CType(0, Byte))
        Me.Label6.Location = New System.Drawing.Point(21, 181)
        Me.Label6.Name = "Label6"
        Me.Label6.Size = New System.Drawing.Size(135, 23)
        Me.Label6.TabIndex = 3
        Me.Label6.Text = "New File Save to"
        '
        'Ori_file_btn
        '
        Me.Ori_file_btn.Location = New System.Drawing.Point(475, 90)
        Me.Ori_file_btn.Name = "Ori_file_btn"
        Me.Ori_file_btn.Size = New System.Drawing.Size(36, 23)
        Me.Ori_file_btn.TabIndex = 2
        Me.Ori_file_btn.Text = "..."
        Me.Ori_file_btn.UseVisualStyleBackColor = True
        '
        'TextBox4
        '
        Me.TextBox4.Location = New System.Drawing.Point(25, 90)
        Me.TextBox4.Name = "TextBox4"
        Me.TextBox4.Size = New System.Drawing.Size(404, 23)
        Me.TextBox4.TabIndex = 1
        '
        'Label5
        '
        Me.Label5.AutoSize = True
        Me.Label5.Font = New System.Drawing.Font("Calibri", 14.25!, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, CType(0, Byte))
        Me.Label5.Location = New System.Drawing.Point(21, 64)
        Me.Label5.Name = "Label5"
        Me.Label5.Size = New System.Drawing.Size(100, 23)
        Me.Label5.TabIndex = 0
        Me.Label5.Text = "Original File"
        '
        'TabPage3
        '
        Me.TabPage3.BackColor = System.Drawing.Color.WhiteSmoke
        Me.TabPage3.Controls.Add(Me.CLose_program)
        Me.TabPage3.Controls.Add(Me.Search_Takeout_btn)
        Me.TabPage3.Controls.Add(Me.partnum_supply_txtbox)
        Me.TabPage3.Controls.Add(Me.Label11)
        Me.TabPage3.Location = New System.Drawing.Point(4, 22)
        Me.TabPage3.Name = "TabPage3"
        Me.TabPage3.Padding = New System.Windows.Forms.Padding(3)
        Me.TabPage3.Size = New System.Drawing.Size(528, 714)
        Me.TabPage3.TabIndex = 2
        Me.TabPage3.Text = "Material Supply"
        '
        'CLose_program
        '
        Me.CLose_program.Font = New System.Drawing.Font("Calibri", 12.0!, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, CType(0, Byte))
        Me.CLose_program.Location = New System.Drawing.Point(168, 423)
        Me.CLose_program.Name = "CLose_program"
        Me.CLose_program.Size = New System.Drawing.Size(167, 80)
        Me.CLose_program.TabIndex = 3
        Me.CLose_program.Text = "Save File and Close"
        Me.CLose_program.UseVisualStyleBackColor = True
        '
        'Search_Takeout_btn
        '
        Me.Search_Takeout_btn.Font = New System.Drawing.Font("Calibri", 12.0!, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, CType(0, Byte))
        Me.Search_Takeout_btn.Location = New System.Drawing.Point(94, 276)
        Me.Search_Takeout_btn.Name = "Search_Takeout_btn"
        Me.Search_Takeout_btn.Size = New System.Drawing.Size(332, 86)
        Me.Search_Takeout_btn.TabIndex = 2
        Me.Search_Takeout_btn.Text = "Search and Take Out"
        Me.Search_Takeout_btn.UseVisualStyleBackColor = True
        '
        'partnum_supply_txtbox
        '
        Me.partnum_supply_txtbox.Font = New System.Drawing.Font("Calibri", 12.0!, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, CType(0, Byte))
        Me.partnum_supply_txtbox.Location = New System.Drawing.Point(178, 135)
        Me.partnum_supply_txtbox.Multiline = True
        Me.partnum_supply_txtbox.Name = "partnum_supply_txtbox"
        Me.partnum_supply_txtbox.Size = New System.Drawing.Size(295, 26)
        Me.partnum_supply_txtbox.TabIndex = 1
        '
        'Label11
        '
        Me.Label11.AutoSize = True
        Me.Label11.Font = New System.Drawing.Font("Calibri", 15.75!, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, CType(0, Byte))
        Me.Label11.Location = New System.Drawing.Point(43, 135)
        Me.Label11.Name = "Label11"
        Me.Label11.Size = New System.Drawing.Size(129, 26)
        Me.Label11.TabIndex = 0
        Me.Label11.Text = "Part Number:"
        '
        'Timer1
        '
        Me.Timer1.Interval = 1000
        '
        'New_Product_Build_Tool
        '
        Me.AutoScaleDimensions = New System.Drawing.SizeF(6.0!, 13.0!)
        Me.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font
        Me.ClientSize = New System.Drawing.Size(605, 763)
        Me.Controls.Add(Me.TabControl1)
        Me.Name = "New_Product_Build_Tool"
        Me.Text = "New Product Build Tool"
        Me.TabControl1.ResumeLayout(False)
        Me.TabPage1.ResumeLayout(False)
        Me.TabPage1.PerformLayout()
        CType(Me.Inventory_list, System.ComponentModel.ISupportInitialize).EndInit()
        Me.TabPage2.ResumeLayout(False)
        Me.TabPage2.PerformLayout()
        CType(Me.PictureBox2, System.ComponentModel.ISupportInitialize).EndInit()
        CType(Me.PictureBox1, System.ComponentModel.ISupportInitialize).EndInit()
        Me.TabPage3.ResumeLayout(False)
        Me.TabPage3.PerformLayout()
        Me.ResumeLayout(False)

    End Sub

    Friend WithEvents TabControl1 As TabControl
    Friend WithEvents TabPage2 As TabPage
    Friend WithEvents TabPage3 As TabPage
    Friend WithEvents Ori_qty_txtbox As TextBox
    Friend WithEvents Label2 As Label
    Friend WithEvents Part_num_txtbox As TextBox
    Friend WithEvents Label1 As Label
    Friend WithEvents Inventory_list As DataGridView
    Friend WithEvents Add_btn As Button
    Friend WithEvents Delete_btn As Button
    Friend WithEvents Update_btn As Button
    Friend WithEvents Save_btn As Button
    Friend WithEvents New_side_btn As Button
    Friend WithEvents New_row_btn As Button
    Friend WithEvents Location_txtbox As TextBox
    Friend WithEvents Label3 As Label
    Friend WithEvents Clear_btn As Button
    Friend WithEvents Lock_chkbox As CheckBox
    Friend WithEvents SaveTo_btn As Button
    Friend WithEvents TextBox5 As TextBox
    Friend WithEvents Label6 As Label
    Friend WithEvents Ori_file_btn As Button
    Friend WithEvents TextBox4 As TextBox
    Friend WithEvents Label5 As Label
    Friend WithEvents Start_btn As Button
    Friend WithEvents Actual_qty_txtbox As TextBox
    Friend WithEvents Label7 As Label
    Friend WithEvents Search_btn As Button
    Friend WithEvents Search_txtbox As TextBox
    Friend WithEvents ProgressBar1 As ProgressBar
    Friend WithEvents Take_out_btn As Button
    Friend WithEvents Column1 As DataGridViewTextBoxColumn
    Friend WithEvents Column2 As DataGridViewTextBoxColumn
    Friend WithEvents Column3 As DataGridViewTextBoxColumn
    Friend WithEvents Column4 As DataGridViewTextBoxColumn
    Friend WithEvents Label10 As Label
    Friend WithEvents Label9 As Label
    Friend WithEvents Label8 As Label
    Friend WithEvents Label4 As Label
    Friend WithEvents Timer1 As Timer
    Friend WithEvents Label11 As Label
    Friend WithEvents partnum_supply_txtbox As TextBox
    Friend WithEvents Search_Takeout_btn As Button
    Friend WithEvents CLose_program As Button
    Public WithEvents TabPage1 As TabPage
    Friend WithEvents Format_2_btn As Button
    Friend WithEvents PictureBox2 As PictureBox
    Friend WithEvents PictureBox1 As PictureBox
End Class
