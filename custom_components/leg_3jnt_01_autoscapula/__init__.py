"""Component Leg 3 joints 01 with Auto-Scapula module

Extension of leg_3jnt_01. Adds a scapula_ctl at the shoulder position that
auto-rotates based on the angle between the shoulder (root) and foot IK target.

The auto-rotation is computed entirely with DG node connections (no constraints):
  - A pointMatrixMult decomposes the ik_ctl world position into scapula_npo
    local space
  - An animCurveUU samples atan(localZ / -localY) to get the swing angle
  - The result is multiplied by autoScapula (0-1) and connected to
    scapula_npo.rotateZ

The leg hierarchy is exactly leg_3jnt_01 — all bones/controls under self.root.
scapula_ctl is a sibling of root_npo, so there is zero double-transformation.
"""

import mgear.pymaya as pm
from mgear.pymaya import datatypes

from mgear.shifter import component

from mgear.core import node, fcurve, applyop, vector, icon
from mgear.core import attribute, transform, primitive

#############################################
# COMPONENT
#############################################


class Component(component.Main):
    """Shifter component Class"""

    # =====================================================
    # OBJECTS
    # =====================================================
    def addObjects(self):
        """Add all the objects needed to create the component."""

        self.setup = primitive.addTransformFromPos(
            self.setupWS, self.getName("WS")
        )
        attribute.lockAttribute(self.setup)

        self.WIP = self.options["mode"]

        self.normal = self.getNormalFromPos(self.guide.apos)

        self.length0 = vector.getDistance(
            self.guide.apos[0], self.guide.apos[1]
        )
        self.length1 = vector.getDistance(
            self.guide.apos[1], self.guide.apos[2]
        )
        self.length2 = vector.getDistance(
            self.guide.apos[2], self.guide.apos[3]
        )
        self.length3 = vector.getDistance(
            self.guide.apos[3], self.guide.apos[4]
        )

        # ── Scapula control ──────────────────────────────────────────────
        # Sibling of root_npo — does NOT parent anything in the leg hierarchy.
        # Its rotation is driven purely by DG connections in addOperators.
        t_root = transform.getTransformFromPos(self.guide.apos[0])

        self.scapula_npo = primitive.addTransform(
            self.root, self.getName("scapula_npo"), t_root
        )
        self.scapula_ctl = self.addCtl(
            self.scapula_npo,
            "scapula_ctl",
            t_root,
            self.color_fk,
            "sphere",
            w=self.length0 * 0.35,
            tp=self.parentCtlTag,
        )
        attribute.setKeyableAttributes(
            self.scapula_ctl, ["tx", "ty", "tz", "rx", "ry", "rz"]
        )
        attribute.setInvertMirror(self.scapula_ctl, ["tx", "ry", "rz"])

        # ── Original leg_3jnt_01 objects (unmodified) ────────────────────

        # 3bones chain
        self.chain3bones = primitive.add2DChain(
            self.setup,
            self.getName("chain3bones%s_jnt"),
            self.guide.apos[0:4],
            self.normal,
            False,
            self.WIP,
        )

        # 2bones chain
        self.chain2bones = primitive.add2DChain(
            self.setup,
            self.getName("chain2bones%s_jnt"),
            self.guide.apos[0:3],
            self.normal,
            False,
            self.WIP,
        )

        # Leg chain
        self.legBones = primitive.add2DChain(
            self.root,
            self.getName("legBones%s_jnt"),
            self.guide.apos[0:4],
            self.normal,
            False,
            True,
        )
        if not self.WIP:
            for b in self.legBones:
                b.attr("drawStyle").set(2)

        # Leg chain FK ref
        self.legBonesFK = primitive.add2DChain(
            self.root,
            self.getName("legFK%s_jnt"),
            self.guide.apos[0:4],
            self.normal,
            False,
            self.WIP,
        )

        # Leg chain IK ref
        self.legBonesIK = primitive.add2DChain(
            self.root,
            self.getName("legIK%s_jnt"),
            self.guide.apos[0:4],
            self.normal,
            False,
            self.WIP,
        )

        # 1 bone chain for upv ref
        self.legChainUpvRef = primitive.add2DChain(
            self.root,
            self.getName("legUpvRef%s_jnt"),
            [self.guide.apos[0], self.guide.apos[3]],
            self.normal,
            False,
            self.WIP,
        )

        # mid joints
        self.mid1_jnt = primitive.addJoint(
            self.legBones[0],
            self.getName("mid1_jnt"),
            self.legBones[1].getMatrix(worldSpace=True),
            self.WIP,
        )
        self.mid1_jnt.attr("radius").set(3)
        self.mid1_jnt.setAttr("jointOrient", 0, 0, 0)

        self.mid2_jnt = primitive.addJoint(
            self.legBones[1],
            self.getName("mid2_jnt"),
            self.legBones[2].getMatrix(worldSpace=True),
            self.WIP,
        )
        self.mid2_jnt.attr("radius").set(3)
        self.mid2_jnt.setAttr("jointOrient", 0, 0, 0)

        # base Controllers
        t = transform.getTransformFromPos(self.guide.apos[0])
        self.root_npo = primitive.addTransform(
            self.root, self.getName("root_npo"), t
        )
        self.root_ctl = self.addCtl(
            self.root_npo,
            "root_ctl",
            t,
            self.color_fk,
            "circle",
            w=self.length0 / 6,
            tp=self.scapula_ctl,
        )
        attribute.lockAttribute(self.root_ctl, ["sx", "sy", "sz", "v"])

        # FK Controllers
        t = transform.getTransformLookingAt(
            self.guide.apos[0], self.guide.apos[1], self.normal, "xz", self.negate
        )
        self.fk0_npo = primitive.addTransform(
            self.root_ctl, self.getName("fk0_npo"), t
        )
        self.fk0_ctl = self.addCtl(
            self.fk0_npo,
            "fk0_ctl",
            t,
            self.color_fk,
            "cube",
            w=self.length0,
            h=self.size * 0.1,
            d=self.size * 0.1,
            po=datatypes.Vector(0.5 * self.length0 * self.n_factor, 0, 0),
            tp=self.root_ctl,
        )
        attribute.setKeyableAttributes(self.fk0_ctl)

        t = transform.getTransformLookingAt(
            self.guide.apos[1], self.guide.apos[2], self.normal, "xz", self.negate
        )
        self.fk1_npo = primitive.addTransform(
            self.fk0_ctl, self.getName("fk1_npo"), t
        )
        self.fk1_ctl = self.addCtl(
            self.fk1_npo,
            "fk1_ctl",
            t,
            self.color_fk,
            "cube",
            w=self.length1,
            h=self.size * 0.1,
            d=self.size * 0.1,
            po=datatypes.Vector(0.5 * self.length1 * self.n_factor, 0, 0),
            tp=self.fk0_ctl,
        )
        attribute.setKeyableAttributes(self.fk1_ctl)

        t = transform.getTransformLookingAt(
            self.guide.apos[2], self.guide.apos[3], self.normal, "xz", self.negate
        )
        self.fk2_npo = primitive.addTransform(
            self.fk1_ctl, self.getName("fk2_npo"), t
        )
        self.fk2_ctl = self.addCtl(
            self.fk2_npo,
            "fk2_ctl",
            t,
            self.color_fk,
            "cube",
            w=self.length2,
            h=self.size * 0.1,
            d=self.size * 0.1,
            po=datatypes.Vector(0.5 * self.length2 * self.n_factor, 0, 0),
            tp=self.fk1_ctl,
        )
        attribute.setKeyableAttributes(self.fk2_ctl)

        t = transform.getTransformLookingAt(
            self.guide.apos[3], self.guide.apos[4], self.normal, "xz", self.negate
        )
        self.fk3_npo = primitive.addTransform(
            self.fk2_ctl, self.getName("fk3_npo"), t
        )
        self.fk3_ctl = self.addCtl(
            self.fk3_npo,
            "fk3_ctl",
            t,
            self.color_fk,
            "cube",
            w=self.length3,
            h=self.size * 0.1,
            d=self.size * 0.1,
            po=datatypes.Vector(0.5 * self.length3 * self.n_factor, 0, 0),
            tp=self.fk2_ctl,
        )
        attribute.setKeyableAttributes(self.fk3_ctl)

        self.fk_ctl = [self.fk0_ctl, self.fk1_ctl, self.fk2_ctl, self.fk3_ctl]
        for x in self.fk_ctl:
            attribute.setInvertMirror(x, ["tx", "ty", "tz"])

        # Mid Controllers
        self.knee_lvl = primitive.addTransform(
            self.root,
            self.getName("knee_lvl"),
            transform.getTransform(self.mid1_jnt),
        )
        self.knee_ctl = self.addCtl(
            self.knee_lvl,
            "knee_ctl",
            transform.getTransform(self.mid1_jnt),
            self.color_ik,
            "sphere",
            w=self.size * 0.2,
            tp=self.root_ctl,
        )
        attribute.setInvertMirror(self.knee_ctl, ["tx", "ty", "tz"])
        attribute.lockAttribute(self.knee_ctl, ["sx", "sy", "sz", "v"])

        self.ankle_lvl = primitive.addTransform(
            self.root,
            self.getName("ankle_lvl"),
            transform.getTransform(self.mid2_jnt),
        )
        self.ankle_ctl = self.addCtl(
            self.ankle_lvl,
            "ankle_ctl",
            transform.getTransform(self.mid2_jnt),
            self.color_ik,
            "sphere",
            w=self.size * 0.2,
            tp=self.knee_ctl,
        )
        attribute.setInvertMirror(self.ankle_ctl, ["tx", "ty", "tz"])
        attribute.lockAttribute(self.ankle_ctl, ["sx", "sy", "sz", "v"])

        # IK Controls
        t_align = transform.getTransformLookingAt(
            self.guide.apos[3], self.guide.apos[4], self.normal, "zx", False
        )
        if self.settings["ikOri"]:
            t = transform.getTransformFromPos(self.guide.pos["foot"])
        else:
            t = t_align

        self.ik_cns = primitive.addTransform(
            self.root_ctl, self.getName("ik_cns"), t
        )
        self.ikcns_ctl = self.addCtl(
            self.ik_cns,
            "ikcns_ctl",
            t,
            self.color_ik,
            "null",
            w=self.size * 0.12,
            tp=self.ankle_ctl,
        )
        attribute.setInvertMirror(self.ikcns_ctl, ["tx"])
        attribute.lockAttribute(self.ikcns_ctl, ["sx", "sy", "sz", "v"])

        self.ik_ctl = self.addCtl(
            self.ikcns_ctl,
            "ik_ctl",
            t,
            self.color_ik,
            "cube",
            w=self.size * 0.12,
            h=self.size * 0.12,
            d=self.size * 0.12,
            tp=self.ikcns_ctl,
        )
        attribute.setKeyableAttributes(self.ik_ctl)
        attribute.setRotOrder(self.ik_ctl, "XZY")
        attribute.setInvertMirror(self.ik_ctl, ["tx", "ry", "rz"])
        attribute.lockAttribute(self.ik_ctl, ["sx", "sy", "sz", "v"])

        # 2 bones ik layer
        self.ik2b_ikCtl_ref = primitive.addTransform(
            self.ik_ctl, self.getName("ik2B_A_ref"), t_align
        )
        self.ik2b_bone_ref = primitive.addTransform(
            self.chain3bones[3], self.getName("ik2B_B_ref"), t_align
        )
        self.ik2b_blend = primitive.addTransform(
            self.ik_ctl, self.getName("ik2B_blend"), t_align
        )

        self.roll_ctl = self.addCtl(
            self.ik2b_blend,
            "roll_ctl",
            t_align,
            self.color_ik,
            "crossarrow",
            w=self.length2 * 0.5 * self.n_factor,
            tp=self.ik_ctl,
        )
        self.ik2b_ik_npo = primitive.addTransform(
            self.roll_ctl,
            self.getName("ik2B_ik_npo"),
            transform.getTransform(self.chain3bones[-1]),
        )
        self.ik2b_ik_ref = primitive.addTransformFromPos(
            self.ik2b_ik_npo,
            self.getName("ik2B_ik_ref"),
            self.guide.pos["ankle"],
        )
        attribute.lockAttribute(
            self.roll_ctl, ["tx", "ty", "tz", "sx", "sy", "sz", "v"]
        )

        # upv
        v = self.guide.apos[2] - self.guide.apos[0]
        v = self.normal ^ v
        v.normalize()
        v *= self.size * 0.5
        v += self.guide.apos[1]

        self.upv_lvl = primitive.addTransformFromPos(
            self.root, self.getName("upv_lvl"), v
        )
        self.upv_cns = primitive.addTransformFromPos(
            self.upv_lvl, self.getName("upv_cns"), v
        )
        self.upv_ctl = self.addCtl(
            self.upv_cns,
            "upv_ctl",
            transform.getTransform(self.upv_cns),
            self.color_ik,
            "diamond",
            w=self.size * 0.12,
            tp=self.ik_ctl,
        )
        attribute.setInvertMirror(self.upv_ctl, ["tx"])
        attribute.setKeyableAttributes(self.upv_ctl, ["tx", "ty", "tz"])

        # Soft IK 3 bones
        t = transform.getTransformLookingAt(
            self.guide.pos["root"], self.guide.pos["foot"], self.x_axis, "zx", False
        )
        self.aim_tra = primitive.addTransform(
            self.root_ctl, self.getName("aimSoftIK"), t
        )
        t = transform.getTransformFromPos(self.guide.pos["foot"])
        self.wristSoftIK = primitive.addTransform(
            self.aim_tra, self.getName("wristSoftIK"), t
        )
        self.softblendLoc = primitive.addTransform(
            self.root, self.getName("softblendLoc"), t
        )

        # Soft IK 2 bones
        t = transform.getTransformLookingAt(
            self.guide.pos["root"], self.guide.pos["ankle"], self.x_axis, "zx", False
        )
        self.aim_tra2 = primitive.addTransform(
            self.root_ctl, self.getName("aimSoftIK2"), t
        )
        t = transform.getTransformFromPos(self.guide.pos["ankle"])
        self.ankleSoftIK = primitive.addTransform(
            self.aim_tra2, self.getName("ankleSoftIK"), t
        )
        self.softblendLoc2 = primitive.addTransform(
            self.root, self.getName("softblendLoc2"), t
        )

        # References
        self.ik_ref = primitive.addTransform(
            self.ik_ctl,
            self.getName("ik_ref"),
            transform.getTransform(self.ik_ctl),
        )
        self.fk_ref = primitive.addTransform(
            self.fk_ctl[3],
            self.getName("fk_ref"),
            transform.getTransform(self.ik_ctl),
        )

        # Twist references
        self.rollRef = primitive.add2DChain(
            self.root,
            self.getName("rollChain"),
            self.guide.apos[:2],
            self.normal,
            False,
            self.WIP,
        )

        self.tws0_loc = primitive.addTransform(
            self.rollRef[0],
            self.getName("tws0_loc"),
            transform.getTransform(self.legBones[0]),
        )
        self.tws0_rot = primitive.addTransform(
            self.tws0_loc,
            self.getName("tws0_rot"),
            transform.getTransform(self.legBones[0]),
        )
        self.tws0_rot.setAttr("sx", 0.001)

        self.tws1_loc = primitive.addTransform(
            self.mid1_jnt,
            self.getName("tws1_loc"),
            transform.getTransform(self.mid1_jnt),
        )
        self.tws1_rot = primitive.addTransform(
            self.tws1_loc,
            self.getName("tws1_rot"),
            transform.getTransform(self.mid1_jnt),
        )
        self.tws1_rot.setAttr("sx", 0.001)

        self.tws2_loc = primitive.addTransform(
            self.mid2_jnt,
            self.getName("tws2_loc"),
            transform.getTransform(self.mid2_jnt),
        )
        self.tws2_rot = primitive.addTransform(
            self.tws2_loc,
            self.getName("tws2_rot"),
            transform.getTransform(self.mid2_jnt),
        )
        self.tws2_rot.setAttr("sx", 0.001)

        self.tws3_loc = primitive.addTransform(
            self.legBones[3],
            self.getName("tws3_loc"),
            transform.getTransform(self.legBones[3]),
        )
        self.tws3_rot = primitive.addTransform(
            self.tws3_loc,
            self.getName("tws3_rot"),
            transform.getTransform(self.legBones[3]),
        )
        self.tws3_rot.setAttr("sx", 0.001)

        self.tws3_drv = primitive.addTransform(
            self.legBones[2],
            self.getName("tws3_drv"),
            transform.getTransform(self.legBones[3]),
        )
        self.tws3_drv.setAttr("sx", 0.001)

        # Divisions
        o_set = self.settings
        self.divisions = o_set["div0"] + o_set["div1"] + o_set["div2"] + 4

        self.div_cns = []
        self.tweak_ctl = []
        tagP = self.parentCtlTag
        for i in range(self.divisions):
            div_cns = primitive.addTransform(
                self.root_ctl, self.getName("div%s_loc" % i)
            )
            self.div_cns.append(div_cns)

            tweak_ctl = self.addCtl(
                div_cns,
                "tweak%s_ctl" % i,
                datatypes.TransformationMatrix(),
                self.color_fk,
                "square",
                w=self.size * 0.15,
                d=self.size * 0.15,
                ro=datatypes.Vector([0, 0, 1.5708]),
                tp=tagP,
            )
            attribute.setKeyableAttributes(tweak_ctl)
            tagP = tweak_ctl
            self.tweak_ctl.append(tweak_ctl)
            self.jnt_pos.append([tweak_ctl, i])

        # End reference
        self.end_ref = primitive.addTransform(
            self.tws3_rot,
            self.getName("end_ref"),
            transform.getTransform(self.legBones[3]),
        )
        for a in "xyz":
            self.end_ref.attr("s%s" % a).set(1.0)
        if self.negate:
            self.end_ref.attr("ry").set(-180.0)

        pos = self.guide.pos["ankle"]
        pos[1] = 0
        self.squash_npo = primitive.addTransformFromPos(
            self.end_ref, self.getName("squashEnd_npo"), pos
        )
        self.squash_scl = primitive.addTransformFromPos(
            self.squash_npo, self.getName("squashEnd_scl"), pos
        )
        self.squash_ref = primitive.addTransform(
            self.squash_scl,
            self.getName("squash_ref"),
            transform.getTransform(self.legBones[3]),
        )
        self.jnt_pos.append([self.squash_ref, "end"])

        # match IK FK references
        self.match_fk0_off = self.add_match_ref(
            self.fk_ctl[1], self.root, "matchFk0_npo", False
        )
        self.match_fk0 = self.add_match_ref(
            self.fk_ctl[0], self.match_fk0_off, "fk0_mth"
        )
        self.match_fk1_off = self.add_match_ref(
            self.fk_ctl[2], self.root, "matchFk1_npo", False
        )
        self.match_fk1 = self.add_match_ref(
            self.fk_ctl[1], self.match_fk1_off, "fk1_mth"
        )
        self.match_fk2_off = self.add_match_ref(
            self.fk_ctl[3], self.root, "matchFk2_npo", False
        )
        self.match_fk2 = self.add_match_ref(
            self.fk_ctl[2], self.match_fk2_off, "fk2_mth"
        )
        self.match_fk3 = self.add_match_ref(
            self.fk_ctl[3], self.legBonesIK[-1], "fk3_mth"
        )
        self.match_ik = self.add_match_ref(self.ik_ctl, self.fk3_ctl, "ik_mth")
        self.match_roll = self.add_match_ref(
            self.roll_ctl, self.fk2_ctl, "roll_mth"
        )
        self.match_ikUpv = self.add_match_ref(
            self.upv_ctl, self.fk0_ctl, "upv_mth"
        )

        self.line_ref = icon.connection_display_curve(
            self.getName("visalRef"), [self.upv_ctl, self.knee_ctl]
        )

    # =====================================================
    # ATTRIBUTES
    # =====================================================
    def addAttributes(self):

        # ── Auto-Scapula ──────────────────────────────────────────────────
        self.autoScapula_att = self.addAnimParam(
            "autoScapula",
            "Auto Scapula",
            "double",
            self.settings["autoScapula"],
            0,
            1,
        )

        # ── Leg attributes (identical to leg_3jnt_01) ─────────────────────
        self.blend_att = self.addAnimParam(
            "blend", "Fk/Ik Blend", "double", self.settings["blend"], 0, 1
        )
        self.fullIK_attr = self.addAnimParam(
            "fullIK", "Full 3 bones IK", "double", self.settings["full3BonesIK"], 0, 1
        )
        self.soft_attr = self.addAnimParam(
            "softIKRange", "Soft IK Range", "double", 0.0001, 0.0001, 100
        )
        self.softSpeed_attr = self.addAnimParam(
            "softIKSpeed", "Soft IK Speed", "double", 2.5, 1.001, 10
        )
        self.stretch_attr = self.addAnimParam(
            "stretch", "Stretch", "double", 0, 0, 1
        )
        self.volume_att = self.addAnimParam(
            "volume", "Volume", "double", 1, 0, 1
        )
        self.roll_att = self.addAnimParam(
            "roll", "Roll", "double", 0, -180, 180
        )
        self.roundnessKnee_att = self.addAnimParam(
            "roundnessKnee", "Roundness Knee", "double", 0, 0, self.size
        )
        self.roundnessAnkle_att = self.addAnimParam(
            "roundnessAnkle", "Roundness Ankle", "double", 0, 0, self.size
        )
        self.boneALenghtMult_attr = self.addAnimParam(
            "boneALenMult", "Bone A Mult", "double", 1
        )
        self.boneBLenghtMult_attr = self.addAnimParam(
            "boneBLenMult", "Bone B Mult", "double", 1
        )
        self.boneCLenghtMult_attr = self.addAnimParam(
            "boneCLenMult", "Bone C Mult", "double", 1
        )
        self.boneALenght_attr = self.addAnimParam(
            "boneALen", "Bone A Length", "double", self.length0, keyable=False
        )
        self.boneBLenght_attr = self.addAnimParam(
            "boneBLen", "Bone B Length", "double", self.length1, keyable=False
        )
        self.boneCLenght_attr = self.addAnimParam(
            "boneCLen", "Bone C Length", "double", self.length2, keyable=False
        )

        if self.settings["ikrefarray"]:
            ref_names = self.get_valid_alias_list(
                self.settings["ikrefarray"].split(",")
            )
            if len(ref_names) > 1:
                self.ikref_att = self.addAnimEnumParam(
                    "ikref", "Ik Ref", 0, ref_names
                )

        if self.settings["upvrefarray"]:
            ref_names = self.get_valid_alias_list(
                self.settings["upvrefarray"].split(",")
            )
            ref_names = ["Auto"] + ref_names
            if len(ref_names) > 1:
                self.upvref_att = self.addAnimEnumParam(
                    "upvref", "UpV Ref", 0, ref_names
                )

        if self.validProxyChannels:
            attribute.addProxyAttribute(
                [
                    self.autoScapula_att,
                    self.blend_att,
                    self.roundnessAnkle_att,
                    self.roundnessKnee_att,
                ],
                [
                    self.scapula_ctl,
                    self.fk0_ctl,
                    self.fk1_ctl,
                    self.fk2_ctl,
                    self.ik_ctl,
                    self.upv_ctl,
                ],
            )
            attribute.addProxyAttribute(
                self.roll_att, [self.ik_ctl, self.upv_ctl]
            )

        self.tweakVis_att = self.addAnimParam(
            "Tweak_vis", "Tweak Vis", "bool", False
        )
        self.foot_squash_att = self.addAnimParam(
            "footSquash", "Foot Squash", "double", 1, 0.01, 1.9
        )

        if self.guide.paramDefs["st_profile"].value:
            self.st_value = self.guide.paramDefs["st_profile"].value
            self.sq_value = self.guide.paramDefs["sq_profile"].value
        else:
            self.st_value = fcurve.getFCurveValues(
                self.settings["st_profile"], self.divisions
            )
            self.sq_value = fcurve.getFCurveValues(
                self.settings["sq_profile"], self.divisions
            )

        self.st_att = [
            self.addSetupParam(
                "stretch_%s" % i, "Stretch %s" % i, "double",
                self.st_value[i], -1, 0,
            )
            for i in range(self.divisions)
        ]
        self.sq_att = [
            self.addSetupParam(
                "squash_%s" % i, "Squash %s" % i, "double",
                self.sq_value[i], 0, 1,
            )
            for i in range(self.divisions)
        ]

        self.resample_att = self.addSetupParam(
            "resample", "Resample", "bool", True
        )
        self.absolute_att = self.addSetupParam(
            "absolute", "Absolute", "bool", False
        )

        defValu = self.chain3bones[1].attr("jointOrientZ").get() / 2
        self.kneeFlipOffset_att = self.addSetupParam(
            "kneeFlipOffset", "Knee Flip Offset", "double", defValu, -180, 180
        )
        defValu = self.chain3bones[2].attr("jointOrientZ").get() / 2
        self.ankleFlipOffset_att = self.addSetupParam(
            "ankleFlipOffset", "Ankle Flip Offset", "double", defValu, -180, 180
        )

    # =====================================================
    # OPERATORS
    # =====================================================
    def addOperators(self):

        # ── Auto-Scapula: pure DG connection, no constraints ──────────────
        #
        # 1. Decompose ik_ctl world matrix to get its world-space position.
        # 2. pointMatrixMult multiplies that position by scapula_npo inverse
        #    world matrix → gives ik_ctl position in scapula local space.
        # 3. The swing angle is atan(localZ / -localY):
        #    -localY is the "rest down" length; localZ is forward deviation.
        #    An animCurveUU samples the true atan function in degrees.
        # 4. Multiply by autoScapula (0-1).
        # 5. Connect to scapula_npo.rotateZ (forward-swing axis for a Y-down
        #    leg where X is the bone primary axis).

        dec_ik = pm.createNode(
            "decomposeMatrix", name=self.getName("scap_decIK")
        )
        pm.connectAttr(
            self.ik_ctl.attr("worldMatrix[0]"), dec_ik + ".inputMatrix"
        )

        pmm = pm.createNode(
            "pointMatrixMult", name=self.getName("scap_pmm")
        )
        pm.connectAttr(dec_ik + ".outputTranslate", pmm + ".inPoint")
        pm.connectAttr(
            self.scapula_npo.attr("worldInverseMatrix[0]"), pmm + ".inMatrix"
        )

        # negate localY so "straight down" is positive (rest pose = positive)
        neg_y = pm.createNode(
            "multiplyDivide", name=self.getName("scap_negY")
        )
        neg_y.attr("operation").set(1)
        neg_y.attr("input2X").set(-1.0)
        pm.connectAttr(pmm + ".outputY", neg_y + ".input1X")

        # ratio = localZ / -localY
        div_ratio = pm.createNode(
            "multiplyDivide", name=self.getName("scap_ratio")
        )
        div_ratio.attr("operation").set(2)
        pm.connectAttr(pmm + ".outputZ", div_ratio + ".input1X")
        pm.connectAttr(neg_y + ".outputX", div_ratio + ".input2X")

        # atan lookup curve: maps ratio → degrees
        atn = pm.createNode("animCurveUU", name=self.getName("scap_atan"))
        atn.attr("preInfinity").set(1)   # linear
        atn.attr("postInfinity").set(1)  # linear
        for u, v in [
            (-10.0, -84.289),
            (-2.0,  -63.435),
            (-1.0,  -45.0),
            (-0.5,  -26.565),
            ( 0.0,    0.0),
            ( 0.5,   26.565),
            ( 1.0,   45.0),
            ( 2.0,   63.435),
            (10.0,   84.289),
        ]:
            pm.setKeyframe(
                atn, float=u, value=v,
                inTangentType="linear", outTangentType="linear"
            )
        pm.connectAttr(div_ratio + ".outputX", atn + ".input")

        # scale by autoScapula attribute
        mul_auto = pm.createNode(
            "multiplyDivide", name=self.getName("scap_mulAuto")
        )
        mul_auto.attr("operation").set(1)
        pm.connectAttr(atn + ".output", mul_auto + ".input1X")
        pm.connectAttr(self.autoScapula_att, mul_auto + ".input2X")

        # connect angle to scapula_npo.rotateZ
        pm.connectAttr(mul_auto + ".outputX", self.scapula_npo + ".rotateX")

        # ── Original leg_3jnt_01 operators (unchanged) ────────────────────

        soft_cond_node = node.createConditionNode(
            self.soft_attr, 0.0001, 4, 0.0001, self.soft_attr
        )
        self.soft_attr_cond = soft_cond_node.outColorR

        if self.settings["ikSolver"]:
            self.ikSolver = "ikRPsolver"
        else:
            pm.mel.eval("ikSpringSolver;")
            self.ikSolver = "ikSpringSolver"

        # 1 bone chain Upv ref
        self.ikHandleUpvRef = primitive.addIkHandle(
            self.root,
            self.getName("ikHandleLegChainUpvRef"),
            self.legChainUpvRef,
            "ikSCsolver",
        )
        pm.pointConstraint(self.ik_ctl, self.ikHandleUpvRef)
        self.relatives_map_upv = {"Auto": self.legChainUpvRef[0]}

        # mid joints
        for xjnt, midJ in zip(
            self.legBones[1:3], [self.mid1_jnt, self.mid2_jnt]
        ):
            node.createPairBlend(None, xjnt, 0.5, 1, midJ)
            pm.connectAttr(xjnt + ".translate", midJ + ".translate", f=True)

        pm.parentConstraint(self.mid1_jnt, self.knee_lvl)
        pm.parentConstraint(self.mid2_jnt, self.ankle_lvl)

        multJnt1_node = node.createMulNode(
            self.boneALenght_attr, self.boneALenghtMult_attr
        )
        multJnt2_node = node.createMulNode(
            self.boneBLenght_attr, self.boneBLenghtMult_attr
        )
        multJnt3_node = node.createMulNode(
            self.boneCLenght_attr, self.boneCLenghtMult_attr
        )

        # IK 3 bones
        self.ikHandle = primitive.addIkHandle(
            self.softblendLoc,
            self.getName("ik3BonesHandle"),
            self.chain3bones,
            "ikRPsolver",
            self.upv_ctl,
        )
        if self.ikSolver == "ikSpringSolver":
            pm.connectAttr(
                "ikSpringSolver.message", self.ikHandle.ikSolver, force=True
            )

        if [
            round(elem, 4)
            for elem in transform.getTranslation(self.chain3bones[1])
        ] != [round(elem, 4) for elem in self.guide.apos[1]]:
            add_nodeTwist = node.createAddNode(180.0, self.roll_att)
        else:
            add_nodeTwist = node.createAddNode(0, self.roll_att)
        if self.negate:
            mulVal = 1
        else:
            mulVal = -1
        node.createMulNode(
            add_nodeTwist + ".output", mulVal, self.ikHandle.attr("twist")
        )

        pm.pointConstraint(self.root_ctl, self.chain3bones[0])

        applyop.aimCns(
            self.aim_tra,
            self.ik_ref,
            axis="zx",
            wupType=4,
            wupVector=[1, 0, 0],
            wupObject=self.root_ctl,
            maintainOffset=False,
        )

        plusTotalLength_node = node.createPlusMinusAverage1D(
            [
                multJnt1_node.attr("outputX"),
                multJnt2_node.attr("outputX"),
                multJnt3_node.attr("outputX"),
            ]
        )
        subtract1_node = node.createPlusMinusAverage1D(
            [plusTotalLength_node.attr("output1D"), self.soft_attr_cond], 2
        )
        distance1_node = node.createDistNode(self.ik_ref, self.aim_tra)
        div1_node = node.createDivNode(1.0, self.rig.global_ctl + ".sx")
        mult1_node = node.createMulNode(
            distance1_node + ".distance", div1_node + ".outputX"
        )
        subtract2_node = node.createPlusMinusAverage1D(
            [mult1_node.attr("outputX"), subtract1_node.attr("output1D")], 2
        )
        div2_node = node.createDivNode(
            subtract2_node + ".output1D", self.soft_attr_cond
        )
        mult2_node = node.createMulNode(-1, div2_node + ".outputX")
        power_node = node.createPowNode(
            self.softSpeed_attr, mult2_node + ".outputX"
        )
        mult3_node = node.createMulNode(
            self.soft_attr_cond, power_node + ".outputX"
        )
        subtract3_node = node.createPlusMinusAverage1D(
            [plusTotalLength_node.attr("output1D"), mult3_node.attr("outputX")], 2
        )
        cond1_node = node.createConditionNode(
            self.soft_attr_cond, 0, 2,
            subtract3_node + ".output1D",
            plusTotalLength_node + ".output1D",
        )
        cond2_node = node.createConditionNode(
            mult1_node + ".outputX",
            subtract1_node + ".output1D",
            2,
            cond1_node + ".outColorR",
            mult1_node + ".outputX",
        )
        pm.connectAttr(cond2_node + ".outColorR", self.wristSoftIK + ".tz")

        pc_node = pm.pointConstraint(
            self.wristSoftIK, self.ik_ref, self.softblendLoc
        )
        node.createReverseNode(
            self.stretch_attr, pc_node + ".target[0].targetWeight"
        )
        pm.connectAttr(
            self.stretch_attr, pc_node + ".target[1].targetWeight", f=True
        )

        distance2_node = node.createDistNode(self.softblendLoc, self.wristSoftIK)
        mult4_node = node.createMulNode(
            distance2_node + ".distance", div1_node + ".outputX"
        )

        for i, mulNode in enumerate([multJnt1_node, multJnt2_node, multJnt3_node]):
            div3_node = node.createDivNode(
                mulNode + ".outputX", plusTotalLength_node + ".output1D"
            )
            mult5_node = node.createMulNode(
                mult4_node + ".outputX", div3_node + ".outputX"
            )
            mult6_node = node.createMulNode(
                self.stretch_attr, mult5_node + ".outputX"
            )
            node.createPlusMinusAverage1D(
                [mulNode.attr("outputX"), mult6_node.attr("outputX")],
                1,
                self.chain3bones[i + 1] + ".tx",
            )

        pm.pointConstraint(self.legBones[3], self.tws3_drv, mo=True)

        # IK 2 bones
        self.ikHandle2 = primitive.addIkHandle(
            self.softblendLoc2,
            self.getName("ik2BonesHandle"),
            self.chain2bones,
            "ikRPsolver",
            self.upv_ctl,
        )
        node.createMulNode(self.roll_att, mulVal, self.ikHandle2.attr("twist"))
        pm.pointConstraint(self.root_ctl, self.chain2bones[0])

        parentc_node = pm.parentConstraint(
            self.ik2b_ikCtl_ref, self.ik2b_bone_ref, self.ik2b_blend
        )
        node.createReverseNode(
            self.fullIK_attr, parentc_node + ".target[0].targetWeight"
        )
        pm.connectAttr(
            self.fullIK_attr, parentc_node + ".target[1].targetWeight", f=True
        )

        applyop.aimCns(
            self.aim_tra2,
            self.ik2b_ik_ref,
            axis="zx",
            wupType=4,
            wupVector=[1, 0, 0],
            wupObject=self.root_ctl,
            maintainOffset=False,
        )

        plusTotalLength_node = node.createPlusMinusAverage1D(
            [multJnt1_node.attr("outputX"), multJnt2_node.attr("outputX")]
        )
        subtract1_node = node.createPlusMinusAverage1D(
            [plusTotalLength_node.attr("output1D"), self.soft_attr_cond], 2
        )
        distance1_node = node.createDistNode(self.ik2b_ik_ref, self.aim_tra2)
        div1_node = node.createDivNode(1, self.rig.global_ctl + ".sx")
        mult1_node = node.createMulNode(
            distance1_node + ".distance", div1_node + ".outputX"
        )
        subtract2_node = node.createPlusMinusAverage1D(
            [mult1_node.attr("outputX"), subtract1_node.attr("output1D")], 2
        )
        div2_node = node.createDivNode(
            subtract2_node + ".output1D", self.soft_attr_cond
        )
        mult2_node = node.createMulNode(-1, div2_node + ".outputX")
        power_node = node.createPowNode(
            self.softSpeed_attr, mult2_node + ".outputX"
        )
        mult3_node = node.createMulNode(
            self.soft_attr_cond, power_node + ".outputX"
        )
        subtract3_node = node.createPlusMinusAverage1D(
            [plusTotalLength_node.attr("output1D"), mult3_node.attr("outputX")], 2
        )
        cond1_node = node.createConditionNode(
            self.soft_attr_cond, 0, 2,
            subtract3_node + ".output1D",
            plusTotalLength_node + ".output1D",
        )
        cond2_node = node.createConditionNode(
            mult1_node + ".outputX",
            subtract1_node + ".output1D",
            2,
            cond1_node + ".outColorR",
            mult1_node + ".outputX",
        )
        pm.connectAttr(cond2_node + ".outColorR", self.ankleSoftIK + ".tz")

        pc_node = pm.pointConstraint(
            self.ankleSoftIK, self.ik2b_ik_ref, self.softblendLoc2
        )
        node.createReverseNode(
            self.stretch_attr, pc_node + ".target[0].targetWeight"
        )
        pm.connectAttr(
            self.stretch_attr, pc_node + ".target[1].targetWeight", f=True
        )

        distance2_node = node.createDistNode(
            self.softblendLoc2, self.ankleSoftIK
        )
        mult4_node = node.createMulNode(
            distance2_node + ".distance", div1_node + ".outputX"
        )

        for i, mulNode in enumerate([multJnt1_node, multJnt2_node]):
            div3_node = node.createDivNode(
                mulNode + ".outputX", plusTotalLength_node + ".output1D"
            )
            mult5_node = node.createMulNode(
                mult4_node + ".outputX", div3_node + ".outputX"
            )
            mult6_node = node.createMulNode(
                self.stretch_attr, mult5_node + ".outputX"
            )
            node.createPlusMinusAverage1D(
                [mulNode.attr("outputX"), mult6_node.attr("outputX")],
                1,
                self.chain2bones[i + 1] + ".tx",
            )

        # IK/FK connections
        for i, x in enumerate(self.fk_ctl):
            pm.parentConstraint(x, self.legBonesFK[i], mo=True)

        for i, x in enumerate([self.chain2bones[0], self.chain2bones[1]]):
            pm.parentConstraint(x, self.legBonesIK[i], mo=True)

        pm.pointConstraint(self.ik2b_ik_ref, self.legBonesIK[2])
        applyop.aimCns(
            self.legBonesIK[2],
            self.roll_ctl,
            axis="xy",
            wupType=4,
            wupVector=[0, 1, 0],
            wupObject=self.legBonesIK[1],
            maintainOffset=False,
        )

        pm.connectAttr(
            self.chain3bones[-1].attr("tx"), self.legBonesIK[-1].attr("tx")
        )
        pm.orientConstraint(self.ik_ref, self.legBonesIK[-1], mo=True)

        node.createMulNode(
            -1, self.chain3bones[-1].attr("tx"), self.ik2b_ik_ref.attr("tx")
        )

        for i, x in enumerate(self.legBones):
            node.createPairBlend(
                self.legBonesFK[i], self.legBonesIK[i], self.blend_att, 1, x
            )

        # Twist references
        self.ikhArmRef, self.tmpCrv = applyop.splineIK(
            self.getName("legRollRef"),
            self.rollRef,
            parent=self.root,
            cParent=self.legBones[0],
        )

        initRound = 0.001
        multVal = 1

        multTangent_node = node.createMulNode(self.roundnessKnee_att, multVal)
        add_node = node.createAddNode(multTangent_node + ".outputX", initRound)
        pm.connectAttr(add_node + ".output", self.tws1_rot.attr("sx"))
        for x in ["translate"]:
            pm.connectAttr(self.knee_ctl.attr(x), self.tws1_loc.attr(x))
        for x in "xy":
            pm.connectAttr(
                self.knee_ctl.attr("r" + x), self.tws1_loc.attr("r" + x)
            )

        multTangent_node = node.createMulNode(self.roundnessAnkle_att, multVal)
        add_node = node.createAddNode(multTangent_node + ".outputX", initRound)
        pm.connectAttr(add_node + ".output", self.tws2_rot.attr("sx"))
        for x in ["translate"]:
            pm.connectAttr(self.ankle_ctl.attr(x), self.tws2_loc.attr(x))
        for x in "xy":
            pm.connectAttr(
                self.ankle_ctl.attr("r" + x), self.tws2_loc.attr("r" + x)
            )

        # Volume
        distA_node = node.createDistNode(self.tws0_loc, self.tws1_loc)
        distB_node = node.createDistNode(self.tws1_loc, self.tws2_loc)
        distC_node = node.createDistNode(self.tws2_loc, self.tws3_loc)
        add_node = node.createAddNode(
            distA_node + ".distance", distB_node + ".distance"
        )
        add_node2 = node.createAddNode(
            distC_node + ".distance", add_node + ".output"
        )
        div_node = node.createDivNode(
            add_node2 + ".output", self.root_ctl.attr("sx")
        )
        dm_node = node.createDecomposeMatrixNode(self.root.attr("worldMatrix"))
        div_node2 = node.createDivNode(
            div_node + ".outputX", dm_node + ".outputScaleX"
        )
        self.volDriver_att = div_node2 + ".outputX"

        pm.connectAttr(self.ankleFlipOffset_att, self.tws2_loc.attr("rz"))
        pm.connectAttr(self.kneeFlipOffset_att, self.tws1_loc.attr("rz"))

        # Divisions
        for i, div_cns in enumerate(self.div_cns):
            subdiv = 45

            if i < (self.settings["div0"] + 2):
                perc = i * 0.333 / (self.settings["div0"] + 1.0)
            elif i < (self.settings["div0"] + self.settings["div1"] + 3):
                perc = 0.333 + (i - self.settings["div0"] - 1) * 0.333 / (
                    self.settings["div1"] + 1.0
                )
            else:
                perc = 0.666 + (
                    i - self.settings["div1"] - self.settings["div0"] - 2.0
                ) * 0.333 / (self.settings["div2"] + 1.0)

            if perc == 0.333:
                perc = 0.3338
            elif perc == 0.666:
                perc = 0.6669
            perc = max(0.001, min(0.999, perc))

            cts = [self.tws0_rot, self.tws1_rot, self.tws2_rot, self.tws3_drv]
            o_node = applyop.gear_rollsplinekine_op(div_cns, cts, perc, subdiv)
            pm.connectAttr(self.resample_att, o_node + ".resample")
            pm.connectAttr(self.absolute_att, o_node + ".absolute")

            o_node = applyop.gear_squashstretch2_op(
                div_cns, None, pm.getAttr(self.volDriver_att), "x"
            )
            pm.connectAttr(self.volume_att, o_node + ".blend")
            pm.connectAttr(self.volDriver_att, o_node + ".driver")
            pm.connectAttr(self.st_att[i], o_node + ".stretch")
            pm.connectAttr(self.sq_att[i], o_node + ".squash")

        pm.orientConstraint(
            self.legBones[3],
            self.tws3_drv,
            skip=["y", "z"],
            maintainOffset=True,
            weight=1,
        )

        # Visibilities
        fkvis_node = node.createReverseNode(self.blend_att)
        for ctrl in self.fk_ctl:
            for shp in ctrl.getShapes():
                pm.connectAttr(fkvis_node + ".outputX", shp.attr("visibility"))
        for ctrl in [self.ik_ctl, self.roll_ctl, self.upv_ctl, self.line_ref]:
            for shp in ctrl.getShapes():
                pm.connectAttr(self.blend_att, shp.attr("visibility"))
        for tweak_ctl in self.tweak_ctl:
            for shp in tweak_ctl.getShapes():
                pm.connectAttr(self.tweakVis_att, shp.attr("visibility"))

        pm.connectAttr(self.rig.global_ctl + ".scale", self.setup + ".scale")

        pm.parentConstraint(self.legBones[0], self.match_fk0_off, mo=True)
        pm.parentConstraint(self.legBones[1], self.match_fk1_off, mo=True)
        pm.parentConstraint(self.legBones[2], self.match_fk2_off, mo=True)

        self.foot_squash_att >> self.squash_scl.sx
        self.foot_squash_att >> self.squash_scl.sz
        rev_nod = node.createReverseNode(self.foot_squash_att)
        node.createPlusMinusAverage1D(
            [rev_nod.outputX, 1], output=self.squash_scl.sy
        )

    # =====================================================
    # CONNECTOR
    # =====================================================
    def setRelation(self):
        self.relatives["root"] = self.legBones[0]
        self.relatives["knee"] = self.legBones[1]
        self.relatives["ankle"] = self.legBones[2]
        self.relatives["foot"] = self.legBones[3]
        self.relatives["eff"] = self.legBones[3]

        self.controlRelatives["root"] = self.scapula_ctl
        self.controlRelatives["knee"] = self.fk1_ctl
        self.controlRelatives["ankle"] = self.fk2_ctl
        self.controlRelatives["foot"] = self.ik_ctl
        self.controlRelatives["eff"] = self.fk3_ctl

        self.jointRelatives["root"] = 0
        self.jointRelatives["knee"] = self.settings["div0"] + 2
        self.jointRelatives["ankle"] = (
            self.settings["div0"] + self.settings["div1"] + 2
        )
        self.jointRelatives["foot"] = len(self.div_cns)
        self.jointRelatives["eff"] = len(self.div_cns)

        self.aliasRelatives["eff"] = "tip"

    def connect_standard(self):
        self.parent.addChild(self.root)
        self.connectRef(self.settings["ikrefarray"], self.ik_cns)
        if self.settings["upvrefarray"]:
            self.connectRef(
                "Auto," + self.settings["upvrefarray"], self.upv_cns, True
            )
