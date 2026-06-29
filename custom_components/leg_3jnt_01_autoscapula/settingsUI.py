import mgear.core.pyqt as gqt
QtGui, QtCore, QtWidgets, wrapInstance = gqt.qt_import()


class Ui_Form(object):
    def setupUi(self, Form):
        Form.setObjectName("Form")
        Form.resize(350, 780)
        self.gridLayout = QtWidgets.QGridLayout(Form)
        self.gridLayout.setObjectName("gridLayout")
        self.groupBox = QtWidgets.QGroupBox(Form)
        self.groupBox.setTitle("")
        self.groupBox.setObjectName("groupBox")
        self.gridLayout_2 = QtWidgets.QGridLayout(self.groupBox)
        self.gridLayout_2.setObjectName("gridLayout_2")
        self.verticalLayout = QtWidgets.QVBoxLayout()
        self.verticalLayout.setObjectName("verticalLayout")
        self.formLayout = QtWidgets.QFormLayout()
        self.formLayout.setObjectName("formLayout")

        # ── Auto Scapula ────────────────────────────────────────────────────
        self.autoScapula_label = QtWidgets.QLabel(self.groupBox)
        self.autoScapula_label.setObjectName("autoScapula_label")
        self.formLayout.setWidget(
            0, QtWidgets.QFormLayout.LabelRole, self.autoScapula_label
        )
        self.horizontalLayout_autoScapula = QtWidgets.QHBoxLayout()
        self.horizontalLayout_autoScapula.setObjectName(
            "horizontalLayout_autoScapula"
        )
        self.autoScapula_slider = QtWidgets.QSlider(self.groupBox)
        self.autoScapula_slider.setMinimumSize(QtCore.QSize(0, 15))
        self.autoScapula_slider.setMaximum(100)
        self.autoScapula_slider.setOrientation(QtCore.Qt.Horizontal)
        self.autoScapula_slider.setObjectName("autoScapula_slider")
        self.horizontalLayout_autoScapula.addWidget(self.autoScapula_slider)
        self.autoScapula_spinBox = QtWidgets.QSpinBox(self.groupBox)
        self.autoScapula_spinBox.setMaximum(100)
        self.autoScapula_spinBox.setObjectName("autoScapula_spinBox")
        self.horizontalLayout_autoScapula.addWidget(self.autoScapula_spinBox)
        self.formLayout.setLayout(
            0,
            QtWidgets.QFormLayout.FieldRole,
            self.horizontalLayout_autoScapula,
        )

        # ── IK/FK Blend ─────────────────────────────────────────────────────
        self.ikfk_label = QtWidgets.QLabel(self.groupBox)
        self.ikfk_label.setObjectName("ikfk_label")
        self.formLayout.setWidget(
            1, QtWidgets.QFormLayout.LabelRole, self.ikfk_label
        )
        self.horizontalLayout_2 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_2.setObjectName("horizontalLayout_2")
        self.ikfk_slider = QtWidgets.QSlider(self.groupBox)
        self.ikfk_slider.setMinimumSize(QtCore.QSize(0, 15))
        self.ikfk_slider.setMaximum(100)
        self.ikfk_slider.setOrientation(QtCore.Qt.Horizontal)
        self.ikfk_slider.setObjectName("ikfk_slider")
        self.horizontalLayout_2.addWidget(self.ikfk_slider)
        self.ikfk_spinBox = QtWidgets.QSpinBox(self.groupBox)
        self.ikfk_spinBox.setMaximum(100)
        self.ikfk_spinBox.setObjectName("ikfk_spinBox")
        self.horizontalLayout_2.addWidget(self.ikfk_spinBox)
        self.formLayout.setLayout(
            1, QtWidgets.QFormLayout.FieldRole, self.horizontalLayout_2
        )

        # ── Full 3 Bones IK ─────────────────────────────────────────────────
        self.full3BonesIK_label = QtWidgets.QLabel(self.groupBox)
        self.full3BonesIK_label.setObjectName("full3BonesIK_label")
        self.formLayout.setWidget(
            2, QtWidgets.QFormLayout.LabelRole, self.full3BonesIK_label
        )
        self.horizontalLayout_3 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_3.setObjectName("horizontalLayout_3")
        self.full3BonesIK_slider = QtWidgets.QSlider(self.groupBox)
        self.full3BonesIK_slider.setMinimumSize(QtCore.QSize(0, 15))
        self.full3BonesIK_slider.setMaximum(100)
        self.full3BonesIK_slider.setOrientation(QtCore.Qt.Horizontal)
        self.full3BonesIK_slider.setObjectName("full3BonesIK_slider")
        self.horizontalLayout_3.addWidget(self.full3BonesIK_slider)
        self.full3BonesIK_spinBox = QtWidgets.QSpinBox(self.groupBox)
        self.full3BonesIK_spinBox.setMaximum(100)
        self.full3BonesIK_spinBox.setObjectName("full3BonesIK_spinBox")
        self.horizontalLayout_3.addWidget(self.full3BonesIK_spinBox)
        self.formLayout.setLayout(
            2, QtWidgets.QFormLayout.FieldRole, self.horizontalLayout_3
        )

        # ── Max Stretch ──────────────────────────────────────────────────────
        self.maxStretch_label = QtWidgets.QLabel(self.groupBox)
        self.maxStretch_label.setObjectName("maxStretch_label")
        self.formLayout.setWidget(
            3, QtWidgets.QFormLayout.LabelRole, self.maxStretch_label
        )
        self.maxStretch_spinBox = QtWidgets.QDoubleSpinBox(self.groupBox)
        self.maxStretch_spinBox.setMinimum(1.0)
        self.maxStretch_spinBox.setSingleStep(0.1)
        self.maxStretch_spinBox.setProperty("value", 1.5)
        self.maxStretch_spinBox.setObjectName("maxStretch_spinBox")
        self.formLayout.setWidget(
            3, QtWidgets.QFormLayout.FieldRole, self.maxStretch_spinBox
        )

        # ── IK Solver ────────────────────────────────────────────────────────
        self.ikSolver_label = QtWidgets.QLabel(self.groupBox)
        self.ikSolver_label.setObjectName("ikSolver_label")
        self.formLayout.setWidget(
            4, QtWidgets.QFormLayout.LabelRole, self.ikSolver_label
        )
        self.ikSolver_comboBox = QtWidgets.QComboBox(self.groupBox)
        self.ikSolver_comboBox.setObjectName("ikSolver_comboBox")
        self.ikSolver_comboBox.addItem("IK Spring")
        self.ikSolver_comboBox.addItem("IK Rotation Plane")
        self.formLayout.setWidget(
            4, QtWidgets.QFormLayout.FieldRole, self.ikSolver_comboBox
        )

        # ── Neutral Rotation ─────────────────────────────────────────────────
        self.neutralRotation_label = QtWidgets.QLabel(self.groupBox)
        self.neutralRotation_label.setObjectName("neutralRotation_label")
        self.formLayout.setWidget(
            5, QtWidgets.QFormLayout.LabelRole, self.neutralRotation_label
        )
        self.neutralRotation_checkBox = QtWidgets.QCheckBox(self.groupBox)
        self.neutralRotation_checkBox.setText("")
        self.neutralRotation_checkBox.setObjectName("neutralRotation_checkBox")
        self.formLayout.setWidget(
            5, QtWidgets.QFormLayout.FieldRole, self.neutralRotation_checkBox
        )

        # ── Divisions ────────────────────────────────────────────────────────
        self.div0_label = QtWidgets.QLabel(self.groupBox)
        self.div0_label.setObjectName("div0_label")
        self.formLayout.setWidget(
            6, QtWidgets.QFormLayout.LabelRole, self.div0_label
        )
        self.div0_spinBox = QtWidgets.QSpinBox(self.groupBox)
        self.div0_spinBox.setMinimum(1)
        self.div0_spinBox.setProperty("value", 2)
        self.div0_spinBox.setObjectName("div0_spinBox")
        self.formLayout.setWidget(
            6, QtWidgets.QFormLayout.FieldRole, self.div0_spinBox
        )

        self.div1_label = QtWidgets.QLabel(self.groupBox)
        self.div1_label.setObjectName("div1_label")
        self.formLayout.setWidget(
            7, QtWidgets.QFormLayout.LabelRole, self.div1_label
        )
        self.div1_spinBox = QtWidgets.QSpinBox(self.groupBox)
        self.div1_spinBox.setMinimum(1)
        self.div1_spinBox.setProperty("value", 2)
        self.div1_spinBox.setObjectName("div1_spinBox")
        self.formLayout.setWidget(
            7, QtWidgets.QFormLayout.FieldRole, self.div1_spinBox
        )

        self.div2_label = QtWidgets.QLabel(self.groupBox)
        self.div2_label.setObjectName("div2_label")
        self.formLayout.setWidget(
            8, QtWidgets.QFormLayout.LabelRole, self.div2_label
        )
        self.div2_spinBox = QtWidgets.QSpinBox(self.groupBox)
        self.div2_spinBox.setMinimum(1)
        self.div2_spinBox.setProperty("value", 2)
        self.div2_spinBox.setObjectName("div2_spinBox")
        self.formLayout.setWidget(
            8, QtWidgets.QFormLayout.FieldRole, self.div2_spinBox
        )

        self.verticalLayout.addLayout(self.formLayout)

        # ── Squash/Stretch profile ───────────────────────────────────────────
        self.squashStretchProfile_pushButton = QtWidgets.QPushButton(
            self.groupBox
        )
        self.squashStretchProfile_pushButton.setObjectName(
            "squashStretchProfile_pushButton"
        )
        self.verticalLayout.addWidget(self.squashStretchProfile_pushButton)

        # ── IK Ref Array ─────────────────────────────────────────────────────
        self.ikRefArray_groupBox = QtWidgets.QGroupBox(self.groupBox)
        self.ikRefArray_groupBox.setObjectName("ikRefArray_groupBox")
        self.ikRefArray_verticalLayout = QtWidgets.QVBoxLayout(
            self.ikRefArray_groupBox
        )
        self.ikRefArray_verticalLayout.setObjectName(
            "ikRefArray_verticalLayout"
        )
        self.ikRefArray_listWidget = QtWidgets.QListWidget(
            self.ikRefArray_groupBox
        )
        self.ikRefArray_listWidget.setDragDropMode(
            QtWidgets.QAbstractItemView.DragDrop
        )
        self.ikRefArray_listWidget.setDefaultDropAction(QtCore.Qt.MoveAction)
        self.ikRefArray_listWidget.setAlternatingRowColors(True)
        self.ikRefArray_listWidget.setObjectName("ikRefArray_listWidget")
        self.ikRefArray_verticalLayout.addWidget(self.ikRefArray_listWidget)
        self.ikRefArray_horizontalLayout = QtWidgets.QHBoxLayout()
        self.ikRefArray_horizontalLayout.setObjectName(
            "ikRefArray_horizontalLayout"
        )
        self.ikRefArrayAdd_pushButton = QtWidgets.QPushButton(
            self.ikRefArray_groupBox
        )
        self.ikRefArrayAdd_pushButton.setObjectName("ikRefArrayAdd_pushButton")
        self.ikRefArray_horizontalLayout.addWidget(
            self.ikRefArrayAdd_pushButton
        )
        self.ikRefArrayRemove_pushButton = QtWidgets.QPushButton(
            self.ikRefArray_groupBox
        )
        self.ikRefArrayRemove_pushButton.setObjectName(
            "ikRefArrayRemove_pushButton"
        )
        self.ikRefArray_horizontalLayout.addWidget(
            self.ikRefArrayRemove_pushButton
        )
        self.ikRefArray_copyRef_pushButton = QtWidgets.QPushButton(
            self.ikRefArray_groupBox
        )
        self.ikRefArray_copyRef_pushButton.setObjectName(
            "ikRefArray_copyRef_pushButton"
        )
        self.ikRefArray_horizontalLayout.addWidget(
            self.ikRefArray_copyRef_pushButton
        )
        self.ikRefArray_verticalLayout.addLayout(
            self.ikRefArray_horizontalLayout
        )
        self.verticalLayout.addWidget(self.ikRefArray_groupBox)

        # ── UpV Ref Array ────────────────────────────────────────────────────
        self.upvRefArray_groupBox = QtWidgets.QGroupBox(self.groupBox)
        self.upvRefArray_groupBox.setObjectName("upvRefArray_groupBox")
        self.upvRefArray_verticalLayout = QtWidgets.QVBoxLayout(
            self.upvRefArray_groupBox
        )
        self.upvRefArray_verticalLayout.setObjectName(
            "upvRefArray_verticalLayout"
        )
        self.upvRefArray_listWidget = QtWidgets.QListWidget(
            self.upvRefArray_groupBox
        )
        self.upvRefArray_listWidget.setDragDropMode(
            QtWidgets.QAbstractItemView.DragDrop
        )
        self.upvRefArray_listWidget.setDefaultDropAction(QtCore.Qt.MoveAction)
        self.upvRefArray_listWidget.setAlternatingRowColors(True)
        self.upvRefArray_listWidget.setObjectName("upvRefArray_listWidget")
        self.upvRefArray_verticalLayout.addWidget(self.upvRefArray_listWidget)
        self.upvRefArray_horizontalLayout = QtWidgets.QHBoxLayout()
        self.upvRefArray_horizontalLayout.setObjectName(
            "upvRefArray_horizontalLayout"
        )
        self.upvRefArrayAdd_pushButton = QtWidgets.QPushButton(
            self.upvRefArray_groupBox
        )
        self.upvRefArrayAdd_pushButton.setObjectName(
            "upvRefArrayAdd_pushButton"
        )
        self.upvRefArray_horizontalLayout.addWidget(
            self.upvRefArrayAdd_pushButton
        )
        self.upvRefArrayRemove_pushButton = QtWidgets.QPushButton(
            self.upvRefArray_groupBox
        )
        self.upvRefArrayRemove_pushButton.setObjectName(
            "upvRefArrayRemove_pushButton"
        )
        self.upvRefArray_horizontalLayout.addWidget(
            self.upvRefArrayRemove_pushButton
        )
        self.upvRefArray_copyRef_pushButton = QtWidgets.QPushButton(
            self.upvRefArray_groupBox
        )
        self.upvRefArray_copyRef_pushButton.setObjectName(
            "upvRefArray_copyRef_pushButton"
        )
        self.upvRefArray_horizontalLayout.addWidget(
            self.upvRefArray_copyRef_pushButton
        )
        self.upvRefArray_verticalLayout.addLayout(
            self.upvRefArray_horizontalLayout
        )
        self.verticalLayout.addWidget(self.upvRefArray_groupBox)

        self.gridLayout_2.addLayout(self.verticalLayout, 0, 0, 1, 1)
        self.gridLayout.addWidget(self.groupBox, 0, 0, 1, 1)

        self.retranslateUi(Form)

        # Signal connections (Qt5+ style, no QtCore.SIGNAL)
        self.autoScapula_slider.sliderMoved.connect(
            self.autoScapula_spinBox.setValue
        )
        self.autoScapula_spinBox.valueChanged.connect(
            self.autoScapula_slider.setValue
        )
        self.ikfk_slider.sliderMoved.connect(self.ikfk_spinBox.setValue)
        self.ikfk_spinBox.valueChanged.connect(self.ikfk_slider.setValue)
        self.full3BonesIK_slider.sliderMoved.connect(
            self.full3BonesIK_spinBox.setValue
        )
        self.full3BonesIK_spinBox.valueChanged.connect(
            self.full3BonesIK_slider.setValue
        )

        QtCore.QMetaObject.connectSlotsByName(Form)

    def retranslateUi(self, Form):
        Form.setWindowTitle(
            gqt.fakeTranslate("Form", "Form", None, -1)
        )
        self.autoScapula_label.setText(
            gqt.fakeTranslate("Form", "Auto Scapula", None, -1)
        )
        self.ikfk_label.setText(
            gqt.fakeTranslate("Form", "IK/FK Blend", None, -1)
        )
        self.full3BonesIK_label.setText(
            gqt.fakeTranslate("Form", "Full 3 Bones IK", None, -1)
        )
        self.maxStretch_label.setText(
            gqt.fakeTranslate("Form", "Max Stretch", None, -1)
        )
        self.ikSolver_label.setText(
            gqt.fakeTranslate("Form", "IK Solver", None, -1)
        )
        self.neutralRotation_label.setText(
            gqt.fakeTranslate("Form", "Neutral Rotation", None, -1)
        )
        self.div0_label.setText(
            gqt.fakeTranslate("Form", "Divisions Upper", None, -1)
        )
        self.div1_label.setText(
            gqt.fakeTranslate("Form", "Divisions Mid", None, -1)
        )
        self.div2_label.setText(
            gqt.fakeTranslate("Form", "Divisions Lower", None, -1)
        )
        self.squashStretchProfile_pushButton.setText(
            gqt.fakeTranslate("Form", "Squash and Stretch Profile", None, -1)
        )
        self.ikRefArray_groupBox.setTitle(
            gqt.fakeTranslate("Form", "IK Reference Array", None, -1)
        )
        self.ikRefArrayAdd_pushButton.setText(
            gqt.fakeTranslate("Form", "Add", None, -1)
        )
        self.ikRefArrayRemove_pushButton.setText(
            gqt.fakeTranslate("Form", "Remove", None, -1)
        )
        self.ikRefArray_copyRef_pushButton.setText(
            gqt.fakeTranslate("Form", "Copy from UpV", None, -1)
        )
        self.upvRefArray_groupBox.setTitle(
            gqt.fakeTranslate("Form", "UpV Reference Array", None, -1)
        )
        self.upvRefArrayAdd_pushButton.setText(
            gqt.fakeTranslate("Form", "Add", None, -1)
        )
        self.upvRefArrayRemove_pushButton.setText(
            gqt.fakeTranslate("Form", "Remove", None, -1)
        )
        self.upvRefArray_copyRef_pushButton.setText(
            gqt.fakeTranslate("Form", "Copy from IK", None, -1)
        )
