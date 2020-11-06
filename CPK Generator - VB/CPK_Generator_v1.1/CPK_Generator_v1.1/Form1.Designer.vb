<Global.Microsoft.VisualBasic.CompilerServices.DesignerGenerated()> _
Partial Class Form1
    Inherits System.Windows.Forms.Form

    'Form overrides dispose to clean up the component list.
    <System.Diagnostics.DebuggerNonUserCode()> _
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
    <System.Diagnostics.DebuggerStepThrough()> _
    Private Sub InitializeComponent()
        Dim resources As System.ComponentModel.ComponentResourceManager = New System.ComponentModel.ComponentResourceManager(GetType(Form1))
        Me.Label4 = New System.Windows.Forms.Label()
        Me.Label3 = New System.Windows.Forms.Label()
        Me.Label2 = New System.Windows.Forms.Label()
        Me.Col_index = New System.Windows.Forms.TextBox()
        Me.Row_index = New System.Windows.Forms.TextBox()
        Me.Label1 = New System.Windows.Forms.Label()
        Me.Exit_button = New System.Windows.Forms.Button()
        Me.Generate = New System.Windows.Forms.Button()
        Me.Save_To = New System.Windows.Forms.Button()
        Me.Browse = New System.Windows.Forms.Button()
        Me.TextBox1 = New System.Windows.Forms.TextBox()
        Me.TextBox2 = New System.Windows.Forms.TextBox()
        Me.SuspendLayout()
        '
        'Label4
        '
        Me.Label4.AutoSize = True
        Me.Label4.BackColor = System.Drawing.Color.Transparent
        Me.Label4.Font = New System.Drawing.Font("Calibri", 12.0!, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, CType(0, Byte))
        Me.Label4.Location = New System.Drawing.Point(428, 173)
        Me.Label4.Name = "Label4"
        Me.Label4.Size = New System.Drawing.Size(14, 19)
        Me.Label4.TabIndex = 22
        Me.Label4.Text = ")"
        '
        'Label3
        '
        Me.Label3.AutoSize = True
        Me.Label3.BackColor = System.Drawing.Color.Transparent
        Me.Label3.Font = New System.Drawing.Font("Calibri", 12.0!, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, CType(0, Byte))
        Me.Label3.Location = New System.Drawing.Point(207, 173)
        Me.Label3.Name = "Label3"
        Me.Label3.Size = New System.Drawing.Size(14, 19)
        Me.Label3.TabIndex = 23
        Me.Label3.Text = "("
        '
        'Label2
        '
        Me.Label2.AutoSize = True
        Me.Label2.BackColor = System.Drawing.Color.Transparent
        Me.Label2.Font = New System.Drawing.Font("Calibri", 12.0!, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, CType(0, Byte))
        Me.Label2.Location = New System.Drawing.Point(318, 179)
        Me.Label2.Name = "Label2"
        Me.Label2.Size = New System.Drawing.Size(13, 19)
        Me.Label2.TabIndex = 21
        Me.Label2.Text = ","
        '
        'Col_index
        '
        Me.Col_index.Font = New System.Drawing.Font("Microsoft Sans Serif", 12.0!, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, CType(0, Byte))
        Me.Col_index.Location = New System.Drawing.Point(337, 168)
        Me.Col_index.Multiline = True
        Me.Col_index.Name = "Col_index"
        Me.Col_index.Size = New System.Drawing.Size(85, 30)
        Me.Col_index.TabIndex = 19
        Me.Col_index.TextAlign = System.Windows.Forms.HorizontalAlignment.Center
        '
        'Row_index
        '
        Me.Row_index.Font = New System.Drawing.Font("Microsoft Sans Serif", 12.0!, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, CType(0, Byte))
        Me.Row_index.Location = New System.Drawing.Point(227, 168)
        Me.Row_index.Multiline = True
        Me.Row_index.Name = "Row_index"
        Me.Row_index.Size = New System.Drawing.Size(85, 30)
        Me.Row_index.TabIndex = 20
        Me.Row_index.TextAlign = System.Windows.Forms.HorizontalAlignment.Center
        '
        'Label1
        '
        Me.Label1.AutoSize = True
        Me.Label1.BackColor = System.Drawing.Color.Transparent
        Me.Label1.Font = New System.Drawing.Font("Calibri", 15.75!, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, CType(0, Byte))
        Me.Label1.Location = New System.Drawing.Point(53, 168)
        Me.Label1.Name = "Label1"
        Me.Label1.Size = New System.Drawing.Size(157, 26)
        Me.Label1.TabIndex = 18
        Me.Label1.Text = "Data starts from:"
        '
        'Exit_button
        '
        Me.Exit_button.Location = New System.Drawing.Point(174, 356)
        Me.Exit_button.Name = "Exit_button"
        Me.Exit_button.Size = New System.Drawing.Size(150, 57)
        Me.Exit_button.TabIndex = 17
        Me.Exit_button.Text = "Exit"
        Me.Exit_button.UseVisualStyleBackColor = True
        '
        'Generate
        '
        Me.Generate.Location = New System.Drawing.Point(154, 226)
        Me.Generate.Name = "Generate"
        Me.Generate.Size = New System.Drawing.Size(196, 101)
        Me.Generate.TabIndex = 16
        Me.Generate.Text = "Generate CPK"
        Me.Generate.UseVisualStyleBackColor = True
        '
        'Save_To
        '
        Me.Save_To.Location = New System.Drawing.Point(265, 95)
        Me.Save_To.Name = "Save_To"
        Me.Save_To.Size = New System.Drawing.Size(166, 48)
        Me.Save_To.TabIndex = 15
        Me.Save_To.Text = "Save To"
        Me.Save_To.UseVisualStyleBackColor = True
        '
        'Browse
        '
        Me.Browse.Location = New System.Drawing.Point(53, 95)
        Me.Browse.Name = "Browse"
        Me.Browse.Size = New System.Drawing.Size(166, 48)
        Me.Browse.TabIndex = 14
        Me.Browse.Text = "Browse the File"
        Me.Browse.UseVisualStyleBackColor = True
        '
        'TextBox1
        '
        Me.TextBox1.Location = New System.Drawing.Point(265, 48)
        Me.TextBox1.Name = "TextBox1"
        Me.TextBox1.Size = New System.Drawing.Size(166, 20)
        Me.TextBox1.TabIndex = 13
        '
        'TextBox2
        '
        Me.TextBox2.Location = New System.Drawing.Point(53, 48)
        Me.TextBox2.Name = "TextBox2"
        Me.TextBox2.Size = New System.Drawing.Size(166, 20)
        Me.TextBox2.TabIndex = 12
        '
        'Form1
        '
        Me.AutoScaleDimensions = New System.Drawing.SizeF(6.0!, 13.0!)
        Me.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font
        Me.BackgroundImage = CType(resources.GetObject("$this.BackgroundImage"), System.Drawing.Image)
        Me.ClientSize = New System.Drawing.Size(496, 464)
        Me.Controls.Add(Me.Label4)
        Me.Controls.Add(Me.Label3)
        Me.Controls.Add(Me.Label2)
        Me.Controls.Add(Me.Col_index)
        Me.Controls.Add(Me.Row_index)
        Me.Controls.Add(Me.Label1)
        Me.Controls.Add(Me.Exit_button)
        Me.Controls.Add(Me.Generate)
        Me.Controls.Add(Me.Save_To)
        Me.Controls.Add(Me.Browse)
        Me.Controls.Add(Me.TextBox1)
        Me.Controls.Add(Me.TextBox2)
        Me.Name = "Form1"
        Me.Text = "Form1"
        Me.ResumeLayout(False)
        Me.PerformLayout()

    End Sub

    Friend WithEvents Label4 As Label
    Friend WithEvents Label3 As Label
    Friend WithEvents Label2 As Label
    Friend WithEvents Col_index As TextBox
    Friend WithEvents Row_index As TextBox
    Friend WithEvents Label1 As Label
    Friend WithEvents Exit_button As Button
    Friend WithEvents Generate As Button
    Friend WithEvents Save_To As Button
    Friend WithEvents Browse As Button
    Friend WithEvents TextBox1 As TextBox
    Friend WithEvents TextBox2 As TextBox
End Class
