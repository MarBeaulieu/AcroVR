using UnityEngine;
using UnityEngine.UI;
using XDA;

// =================================================================================================================================================================
/// <summary> Script principal de la scène TestXSens, du logiciel AcroVR. </summary>

public class MainTestXSens : MonoBehaviour
{
	public XSensInterface m_XSens;
	public S2M_model modelS2M;
	public GameObject panelTestMessages;

	XSensData xSdataTrial;
	XSensData xSdataCalib;

	System.Diagnostics.Stopwatch timeStamp = new System.Diagnostics.Stopwatch();

	string pathToTemplate = @"Assets\StreamingAssets\Somersault_IMUs.s2mMod";
	string pathToModel = @"Assets\StreamingAssets\Model.s2mMod";

	Text textTestMessages;
	int runState = 9;					// État de fonctionnement du logiciel
	bool calibDone = false;
	float timeBegin;
	float timeEnd;
	int numberFramesCalib = 50;
	int debugFrameN = 0;
	bool debugNoXSens = true;

	// =================================================================================================================================================================
	/// <summary> Initialisation du script. </summary>

	void Start()
	{
	}

	// =================================================================================================================================================================
	/// <summary> Exécution de la fonction à chaque frame. </summary>

	void Update()
	{
		// Vérifier que la station XSens Awinda est prête et que tous les senseurs XSens sont prêt à être utiliser

		if (!debugNoXSens)
			if (!m_XSens.canUseTheSystem())
				return;

		// Afficher le message de calibration à l'écran

		if (runState <= 0)
		{
			timeBegin = Time.time;
			timeEnd = timeBegin + 5;
			panelTestMessages.SetActive(true);
			textTestMessages.text = string.Format("Calibration dans {0:F0}s ...", timeEnd - timeBegin);
			runState = 1;
		}

		// Attendre le début de la calibration
		// Initialisation du minuteur et supprimer tous les anciens données de XSens (matrice de rotation)

		else if (runState == 1)
		{
			float timeElapsed = timeEnd - Time.time;
			textTestMessages.text = string.Format("Calibration dans {0:F0}s ...", timeElapsed);
			if (timeElapsed < 0)
			{
				timeBegin = Time.time;
				timeEnd = timeBegin + 2;
				textTestMessages.text = "Calibration ...";
				if (!debugNoXSens)
				{
					timeStamp.Reset();
					timeStamp.Start();
				}
				runState = 2;
			}
		}

		// Calibration

		else if (runState == 2)
		{
			if (timeEnd - Time.time < 0)
			{
				timeBegin = Time.time;
				timeEnd = timeBegin + 2;
				textTestMessages.text = "Calibration terminé";
				xSdataCalib.writeTrialToBinary(@"C:\Devel\AcroVR Data\Calib.xSd");
				runState = 3;
			}
			Calibration();
		}

		// Calibration terminé, attendre avant de supprimer le message de calibration

		else if (runState == 3 && timeEnd - Time.time < 0)
		{
			timeBegin = Time.time;
			timeEnd = timeBegin + 10;
			panelTestMessages.SetActive(false);
			runState = 4;
		}

		// Message de calibration supprimer
		// Afficher la silhouette selon les déplacements des senseurs XSens
		// Afficher un message quand l'essai est terminé

		else if (runState == 4)
		{
			if (timeEnd - Time.time < 0)
			{
				timeBegin = Time.time;
				timeEnd = timeBegin + 2;
				panelTestMessages.SetActive(true);
				textTestMessages.text = "Essai terminé ...";
				xSdataTrial.writeTrialToBinary(@"C:\Devel\AcroVR Data\Trial.xSd");
				runState = 5;
			}
			DisplayAnimation();
		}

		// Fin affichage de la silhouette, attendre avant de supprimer le message de fin d'essai

		else if (runState == 5 && timeEnd - Time.time < 0)
		{
			panelTestMessages.SetActive(false);
			runState = 9;
		}
	}

	// =================================================================================================================================================================
	/// <summary> Exécuter le test. </summary>

	public void StartTest()
	{
		xSdataCalib = new XSensData("");
		xSdataTrial = new XSensData("");

		// Initialisation de l'accès aux senseurs XSens (Centrales ou IMU = Inertia Measurement Unit)

		if (!debugNoXSens)
			m_XSens.prepareSystem();
		else
		{
			xSdataCalib.readTrialFromBinary(@"C:\Devel\AcroVR Data\CalibDebug.xSd");
			xSdataTrial.readTrialFromBinary(@"C:\Devel\AcroVR Data\TrialDebug.xSd");
		}

		// On utilise le mode Gesticulation pour le moment

		AnimationF.Instance.playMode = MainParameters.Instance.languages.Used.animatorPlayModeSimulation;

		// Initialisation du modèle Lagrangien

		LagrangianModelSimple lagrangianModelSimple = new LagrangianModelSimple();
		MainParameters.Instance.joints.lagrangianModel = lagrangianModelSimple.GetParameters;

		// Initialisation du message de calibration

		textTestMessages = panelTestMessages.GetComponentInChildren<Text>();

		calibDone = false;
		debugFrameN = 0;
		runState = 0;
	}

	// =================================================================================================================================================================
	/// <summary> Calibration. </summary>

	void Calibration()
	{
		// Lire et conserver les matrices de rotation pour tous les senseurs XSens, seulement si tous les senseurs ont rapporté des données

		XsMatrix[] IMUs;
		if (!debugNoXSens)
		{
			IMUs = m_XSens.dataMatrix();
			foreach (XsMatrix IMU in IMUs)
				if (IMU == null)
					return;
			xSdataCalib.Add(timeStamp.ElapsedMilliseconds, IMUs);
		}

		// Faire la calibration
		// Modifier le modèle S2M selon les données lues

		if (!calibDone && xSdataCalib.nFrame() >= numberFramesCalib)
		{
			modelS2M.createModelFromStaticXsens(xSdataCalib.getData(), pathToModel, pathToTemplate);
			modelS2M.LoadModel(new System.Text.StringBuilder(pathToModel));
			calibDone = true;
		}
	}

	// =================================================================================================================================================================
	/// <summary> Afficher la silhouette pour un frame. </summary>

	void DisplayAnimation()
	{
		// Lire et conserver les matrices de rotation pour tous les senseurs XSens, seulement si tous les senseurs ont rapporté des données

		XsMatrix[] IMUs;
		if (!debugNoXSens)
		{
			IMUs = m_XSens.dataMatrix();
			foreach (XsMatrix IMU in IMUs)
				if (IMU == null)
					return;
			xSdataTrial.Add(timeStamp.ElapsedMilliseconds, IMUs);
		}
		else
		{
			IMUs = xSdataTrial.getData(debugFrameN);
			debugFrameN++;
		}

		// Passer les données dans un filtre de Kalman étendu
		// Convertir les données BioRBD en Humans
		// Calculer la dimension du volume utilisé à l'écran
		// Afficher la silhouette

		modelS2M.KalmanReconsIMU(IMUs);

		int nDDLBioRBD = modelS2M.getQ().Length;
		double[] qBioRBD = new double[nDDLBioRBD];
		double[] qdotBioRBD = new double[nDDLBioRBD];
		qBioRBD = modelS2M.getQ();
		qdotBioRBD = modelS2M.getQdot();

		double[] xTBioRBD = new double[nDDLBioRBD * 2];
		for (int i = 0; i < nDDLBioRBD; i++)
		{
			xTBioRBD[i] = qBioRBD[i];
			xTBioRBD[i + nDDLBioRBD] = qdotBioRBD[i];
		}

		DoSimulation.xTFrame0 = ConvertHumansBioRBD.Biorbd2Humans(xTBioRBD);             // Convertir les DDL du modèle BioRBD vers le modèle Humans

		float[,] q1;
		q1 = new float[MainParameters.Instance.joints.lagrangianModel.nDDL, 1];
		for (int i = 0; i < MainParameters.Instance.joints.lagrangianModel.nDDL; i++)
			q1[i, 0] = (float)DoSimulation.xTFrame0[i];

		AnimationF.Instance.InitScreenVolumeDimension(q1);

		AnimationF.Instance.Play(0, 1);
		AnimationF.Instance.PlayOneFrame();                     // Affichage de la silhouette
		AnimationF.Instance.PlayEnd();                          // Fin de l'animation
	}

		//// =================================================================================================================================================================
		///// <summary> Conserver les données XSens dans un fichier. </summary>

		//void SaveDataFile()
		//{
		//	string fileLines = string.Format(
		//		"Duration: {0}{1}Condition: {2}{3}VerticalSpeed: {4:0.000}{5}AnteroposteriorSpeed: {6:0.000}{7}SomersaultSpeed: {8:0.000}{9}TwistSpeed: {10:0.000}{11}Tilt: {12:0.000}{13}Rotation: {14:0.000}{15}{16}",
		//		MainParameters.Instance.joints.duration, System.Environment.NewLine,
		//		MainParameters.Instance.joints.condition, System.Environment.NewLine,
		//		MainParameters.Instance.joints.takeOffParam.verticalSpeed, System.Environment.NewLine,
		//		MainParameters.Instance.joints.takeOffParam.anteroposteriorSpeed, System.Environment.NewLine,
		//		MainParameters.Instance.joints.takeOffParam.somersaultSpeed, System.Environment.NewLine,
		//		MainParameters.Instance.joints.takeOffParam.twistSpeed, System.Environment.NewLine,
		//		MainParameters.Instance.joints.takeOffParam.tilt, System.Environment.NewLine,
		//		MainParameters.Instance.joints.takeOffParam.rotation, System.Environment.NewLine, System.Environment.NewLine);

		//	fileLines = string.Format("{0}Nodes{1}DDL, name, interpolation (type, numIntervals, slopes), T, Q{2}", fileLines, System.Environment.NewLine, System.Environment.NewLine);

		//	for (int i = 0; i < MainParameters.Instance.joints.nodes.Length; i++)
		//	{
		//		fileLines = string.Format("{0}{1}:{2}:{3},{4},{5:0.000000},{6:0.000000}:", fileLines, i + 1, MainParameters.Instance.joints.nodes[i].name, MainParameters.Instance.joints.nodes[i].interpolation.type,
		//			MainParameters.Instance.joints.nodes[i].interpolation.numIntervals, MainParameters.Instance.joints.nodes[i].interpolation.slope[0], MainParameters.Instance.joints.nodes[i].interpolation.slope[1]);
		//		for (int j = 0; j < MainParameters.Instance.joints.nodes[i].T.Length; j++)
		//		{
		//			if (j < MainParameters.Instance.joints.nodes[i].T.Length - 1)
		//				fileLines = string.Format("{0}{1:0.000000},", fileLines, MainParameters.Instance.joints.nodes[i].T[j]);
		//			else
		//				fileLines = string.Format("{0}{1:0.000000}:", fileLines, MainParameters.Instance.joints.nodes[i].T[j]);
		//		}
		//		for (int j = 0; j < MainParameters.Instance.joints.nodes[i].Q.Length; j++)
		//		{
		//			if (j < MainParameters.Instance.joints.nodes[i].Q.Length - 1)
		//				fileLines = string.Format("{0}{1:0.000000},", fileLines, MainParameters.Instance.joints.nodes[i].Q[j]);
		//			else
		//				fileLines = string.Format("{0}{1:0.000000}:{2}", fileLines, MainParameters.Instance.joints.nodes[i].Q[j], System.Environment.NewLine);
		//		}
		//	}

		//	System.IO.File.WriteAllText(fileName, fileLines);
		//}
	}
