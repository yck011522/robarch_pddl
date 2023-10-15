(define (problem CantiBoxMid_process)
	(:domain clamp_transfer)
	(:objects CL3 CL3M PG1000 PG1500 PG500 SL1_G200 b0 b1 b10 b11 b12 b13 b14 b15 b16 b17 b18 b19 b2 b3 b4 b5 b6 b7 b8 b9 c1 c2 c3 c4 g1 g2 g3 s4)
	(:init 
		(AssemblyByClampingMethod b10)
		(AssemblyByClampingMethod b11)
		(AssemblyByClampingMethod b12)
		(AssemblyByClampingMethod b14)
		(AssemblyByClampingMethod b16)
		(AssemblyByClampingMethod b18)
		(AssemblyByClampingMethod b2)
		(AssemblyByClampingMethod b3)
		(AssemblyByClampingMethod b4)
		(AssemblyByClampingMethod b5)
		(AssemblyByClampingMethod b6)
		(AssemblyByClampingMethod b7)
		(AssemblyByClampingMethod b8)
		(AssemblyByClampingMethod b9)
		(AssemblyByGroundConnection b0)
		(AssemblyByGroundConnection b1)
		(AssemblyByScrewingMethod b13)
		(AssemblyByScrewingMethod b15)
		(AssemblyByScrewingMethod b17)
		(AssemblyByScrewingMethod b19)
		(Beam b0)
		(Beam b1)
		(Beam b10)
		(Beam b11)
		(Beam b12)
		(Beam b13)
		(Beam b14)
		(Beam b15)
		(Beam b16)
		(Beam b17)
		(Beam b18)
		(Beam b19)
		(Beam b2)
		(Beam b3)
		(Beam b4)
		(Beam b5)
		(Beam b6)
		(Beam b7)
		(Beam b8)
		(Beam b9)
		(BeamAtStorage b0)
		(BeamAtStorage b1)
		(BeamAtStorage b10)
		(BeamAtStorage b11)
		(BeamAtStorage b12)
		(BeamAtStorage b13)
		(BeamAtStorage b14)
		(BeamAtStorage b15)
		(BeamAtStorage b16)
		(BeamAtStorage b17)
		(BeamAtStorage b18)
		(BeamAtStorage b19)
		(BeamAtStorage b2)
		(BeamAtStorage b3)
		(BeamAtStorage b4)
		(BeamAtStorage b5)
		(BeamAtStorage b6)
		(BeamAtStorage b7)
		(BeamAtStorage b8)
		(BeamAtStorage b9)
		(BeamNeedsGripperType b0 PG1500)
		(BeamNeedsGripperType b1 PG1500)
		(BeamNeedsGripperType b10 PG1500)
		(BeamNeedsGripperType b11 PG1500)
		(BeamNeedsGripperType b12 PG1500)
		(BeamNeedsGripperType b13 SL1_G200)
		(BeamNeedsGripperType b14 PG1500)
		(BeamNeedsGripperType b15 SL1_G200)
		(BeamNeedsGripperType b16 PG1500)
		(BeamNeedsGripperType b17 SL1_G200)
		(BeamNeedsGripperType b18 PG1500)
		(BeamNeedsGripperType b19 SL1_G200)
		(BeamNeedsGripperType b2 PG1000)
		(BeamNeedsGripperType b3 PG1000)
		(BeamNeedsGripperType b4 PG1500)
		(BeamNeedsGripperType b5 PG1500)
		(BeamNeedsGripperType b6 PG1500)
		(BeamNeedsGripperType b7 PG1500)
		(BeamNeedsGripperType b8 PG1500)
		(BeamNeedsGripperType b9 PG1500)
		(Clamp c1)
		(Clamp c2)
		(Clamp c3)
		(Clamp c4)
		(ClampAtStorage c1)
		(ClampAtStorage c2)
		(ClampAtStorage c3)
		(ClampAtStorage c4)
		(ClampOfType c1 CL3)
		(ClampOfType c2 CL3)
		(ClampOfType c3 CL3M)
		(ClampOfType c4 CL3M)
		(Gripper g1)
		(Gripper g2)
		(Gripper g3)
		(Gripper s4)
		(GripperAtStorage g1)
		(GripperAtStorage g2)
		(GripperAtStorage g3)
		(GripperAtStorage s4)
		(GripperOfType g1 PG500)
		(GripperOfType g2 PG1000)
		(GripperOfType g3 PG1500)
		(GripperOfType s4 SL1_G200)
		(Joint b0 b15)
		(Joint b0 b2)
		(Joint b0 b5)
		(Joint b1 b17)
		(Joint b1 b3)
		(Joint b1 b6)
		(Joint b10 b19)
		(Joint b11 b13)
		(Joint b12 b13)
		(Joint b14 b15)
		(Joint b16 b17)
		(Joint b18 b19)
		(Joint b2 b10)
		(Joint b2 b14)
		(Joint b2 b18)
		(Joint b2 b4)
		(Joint b2 b8)
		(Joint b3 b10)
		(Joint b3 b16)
		(Joint b3 b18)
		(Joint b3 b4)
		(Joint b3 b9)
		(Joint b4 b19)
		(Joint b5 b11)
		(Joint b5 b12)
		(Joint b5 b14)
		(Joint b5 b7)
		(Joint b5 b8)
		(Joint b6 b11)
		(Joint b6 b12)
		(Joint b6 b16)
		(Joint b6 b7)
		(Joint b6 b9)
		(Joint b7 b13)
		(Joint b8 b15)
		(Joint b9 b17)
		(JointNeedsClampType b0 b2 CL3)
		(JointNeedsClampType b0 b5 CL3M)
		(JointNeedsClampType b1 b3 CL3M)
		(JointNeedsClampType b1 b6 CL3)
		(JointNeedsClampType b2 b10 CL3M)
		(JointNeedsClampType b2 b14 CL3M)
		(JointNeedsClampType b2 b18 CL3)
		(JointNeedsClampType b2 b4 CL3)
		(JointNeedsClampType b2 b8 CL3)
		(JointNeedsClampType b3 b10 CL3)
		(JointNeedsClampType b3 b16 CL3)
		(JointNeedsClampType b3 b18 CL3)
		(JointNeedsClampType b3 b4 CL3M)
		(JointNeedsClampType b3 b9 CL3M)
		(JointNeedsClampType b5 b11 CL3M)
		(JointNeedsClampType b5 b12 CL3M)
		(JointNeedsClampType b5 b14 CL3M)
		(JointNeedsClampType b5 b7 CL3)
		(JointNeedsClampType b5 b8 CL3)
		(JointNeedsClampType b6 b11 CL3)
		(JointNeedsClampType b6 b12 CL3)
		(JointNeedsClampType b6 b16 CL3)
		(JointNeedsClampType b6 b7 CL3M)
		(JointNeedsClampType b6 b9 CL3M))
	(:goal (and (BeamAtAssembled b0) (BeamAtAssembled b1) (BeamAtAssembled b2) (BeamAtAssembled b3) (BeamAtAssembled b4) (BeamAtAssembled b5) (BeamAtAssembled b6) (BeamAtAssembled b7) (BeamAtAssembled b8) (BeamAtAssembled b9) (BeamAtAssembled b10) (BeamAtAssembled b11) (BeamAtAssembled b12) (BeamAtAssembled b13) (BeamAtAssembled b14) (BeamAtAssembled b15) (BeamAtAssembled b16) (BeamAtAssembled b17) (BeamAtAssembled b18) (BeamAtAssembled b19) (BeamAtAssembled b0) (BeamAtAssembled b1) (BeamAtAssembled b2) (BeamAtAssembled b3) (BeamAtAssembled b4) (BeamAtAssembled b5) (BeamAtAssembled b6) (BeamAtAssembled b7) (BeamAtAssembled b8) (BeamAtAssembled b9) (BeamAtAssembled b10) (BeamAtAssembled b11) (BeamAtAssembled b12) (BeamAtAssembled b13) (BeamAtAssembled b14) (BeamAtAssembled b15) (BeamAtAssembled b16) (BeamAtAssembled b17) (BeamAtAssembled b18) (BeamAtAssembled b19) (BeamAtAssembled b0) (BeamAtAssembled b1) (BeamAtAssembled b2) (BeamAtAssembled b3) (BeamAtAssembled b4) (BeamAtAssembled b5) (BeamAtAssembled b6) (BeamAtAssembled b7) (BeamAtAssembled b8) (BeamAtAssembled b9) (BeamAtAssembled b10) (BeamAtAssembled b11) (BeamAtAssembled b12) (BeamAtAssembled b13) (BeamAtAssembled b14) (BeamAtAssembled b15) (BeamAtAssembled b16) (BeamAtAssembled b17) (BeamAtAssembled b18) (BeamAtAssembled b19) (GripperAtStorage g1) (GripperAtStorage g2) (GripperAtStorage g3) (GripperAtStorage s4) (BeamAtAssembled b0) (BeamAtAssembled b1) (BeamAtAssembled b2) (BeamAtAssembled b3) (BeamAtAssembled b4) (BeamAtAssembled b5) (BeamAtAssembled b6) (BeamAtAssembled b7) (BeamAtAssembled b8) (BeamAtAssembled b9) (BeamAtAssembled b10) (BeamAtAssembled b11) (BeamAtAssembled b12) (BeamAtAssembled b13) (BeamAtAssembled b14) (BeamAtAssembled b15) (BeamAtAssembled b16) (BeamAtAssembled b17) (BeamAtAssembled b18) (BeamAtAssembled b19) (GripperAtStorage g1) (GripperAtStorage g2) (GripperAtStorage g3) (GripperAtStorage s4) (BeamAtAssembled b0) (BeamAtAssembled b1) (BeamAtAssembled b2) (BeamAtAssembled b3) (BeamAtAssembled b4) (BeamAtAssembled b5) (BeamAtAssembled b6) (BeamAtAssembled b7) (BeamAtAssembled b8) (BeamAtAssembled b9) (BeamAtAssembled b10) (BeamAtAssembled b11) (BeamAtAssembled b12) (BeamAtAssembled b13) (BeamAtAssembled b14) (BeamAtAssembled b15) (BeamAtAssembled b16) (BeamAtAssembled b17) (BeamAtAssembled b18) (BeamAtAssembled b19) (GripperAtStorage g1) (GripperAtStorage g2) (GripperAtStorage g3) (GripperAtStorage s4) (ClampAtStorage c1) (ClampAtStorage c2) (ClampAtStorage c3) (ClampAtStorage c4))))
