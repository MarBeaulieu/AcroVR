using System;
using System.IO;
using System.Text;
using System.Linq;
using UnityEngine;
using UnityEngine.UI;
using System.Collections;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using XDA;

public class S2M_model : MonoBehaviour
{
	const string dllpath = "S2M.dll";
	[DllImport(dllpath)] static extern IntPtr c_s2mMusculoSkeletalModel(StringBuilder pathToModel);
	[DllImport(dllpath)] static extern void c_deleteS2mMusculoSkeletalModel(IntPtr model);
	[DllImport(dllpath)] static extern void c_writeS2mMusculoSkeletalModel(IntPtr model, StringBuilder path);

	// IMUs functions
	[DllImport(dllpath)] static extern int c_nIMUs(IntPtr model);
	[DllImport(dllpath)] static extern void c_addIMU(IntPtr model, IntPtr imuRT, StringBuilder name, StringBuilder parent, bool isTechnical = true, bool isAnatomical = true);
	[DllImport(dllpath)] static extern void c_meanIMU(IntPtr imuRT, int nFrame, IntPtr imuRT_mean);
	[DllImport(dllpath)] static extern IntPtr c_s2mKalmanReconsIMU(IntPtr model, IntPtr QinitialGuess, double freq = 100, double noiseF = 5e-3, double errorF = 1e-10);
	[DllImport(dllpath)] static extern void c_deleteS2mKalmanReconsIMU(IntPtr kalman);
	[DllImport(dllpath)] static extern void c_s2mKalmanReconsIMUstep(IntPtr model, IntPtr kalman, IntPtr imu, IntPtr Q, IntPtr QDo, IntPtr QDDot);

	// Joint related
	[DllImport(dllpath)] static extern void c_globalJCS(IntPtr model, IntPtr Q, IntPtr jcs); 
	[DllImport(dllpath)] static extern void c_projectJCSinParentBaseCoordinate(IntPtr M1, IntPtr M2, IntPtr M3); 
	[DllImport(dllpath)] static extern void c_matrixMultiplication(IntPtr M1, IntPtr M2, IntPtr M3); 
	[DllImport(dllpath)] static extern void c_localJCS(IntPtr model, int i, IntPtr RtOut);  // Return the LCS for segment named segName in parent coordinate system
	[DllImport(dllpath)] static extern void c_alignSpecificAxisWithParentVertical(IntPtr parentRT, IntPtr childRT, int idxAxe, IntPtr rotation);

	// Dof related
	[DllImport(dllpath)] static extern int c_nQ(IntPtr model); 
	[DllImport(dllpath)] static extern int c_nQDot(IntPtr model); 
	[DllImport(dllpath)] static extern int c_nQDDot(IntPtr model); 
	[DllImport(dllpath)] static extern int c_nTau(IntPtr model); 

	// Markers related
	[DllImport(dllpath)] static extern int c_nTags(IntPtr model); 
	[DllImport(dllpath)] static extern void c_Tags(IntPtr model, IntPtr Q, IntPtr mark, bool removeAxis = true, bool updateKin = true); 
	[DllImport(dllpath)] static extern void c_TagsInLocal(IntPtr model, IntPtr mark); 
	[DllImport(dllpath)] static extern void c_addTags(IntPtr model, IntPtr markPos, StringBuilder name, StringBuilder parent, bool isTechnical, bool isAnatomical, StringBuilder axesToRemove);

	// Maths related
	[DllImport(dllpath)] static extern void c_transformMatrixToCardan(IntPtr M, StringBuilder sequence, IntPtr cardanOut);

    // Tous les pointeurs et nombre qui peuvent sauver du temps ou des appels

    public static bool calibre = false;
    bool m_isloaded = false;
	bool m_isStaticDone = false;
	IntPtr m_s2m; // Pointeur sur des modèles
	int m_nQ = 0, m_nQdot = 0, m_nQddot = 0, m_nTags = 0;
	IntPtr m_Q, m_Qdot, m_Qddot; // stocker ou recevoir des coordonnées généralisées

	IntPtr m_mark; // Stocker ou recevoir des position de marqueurs

	IntPtr m_kalman; // Pointeur sur un filtre de kalmanIMU
	static public int nbImuToConnect = 6;
	List<StringBuilder> m_nameIMUs = new List<StringBuilder>(); // Nom du IMU
	List<StringBuilder> m_parentIMUs = new List<StringBuilder>(); // Nom du parent du IMU
	List<int> m_positionIMUs = new List<int>(); // Numéro du marqueur où est le IMU
	List<int> m_parentIMUsIdx = new List<int>(); // Numéro du marqueur où est le IMU
	int m_nIMUs = 0;
	IntPtr m_IMU; // stocker ou recevoir des matrices d'orientation des centrales inertielles
    double[] Q;
    double[] Qdot;
    double[] Qddot;

    public void Awake()
    {
        //Variables to create a new Biorbd or S2M model from a template

        //Names of the IMUs
        m_nameIMUs.Add(new StringBuilder("Pelvis_IMU"));
        m_nameIMUs.Add(new StringBuilder("Head_IMU"));
        m_nameIMUs.Add(new StringBuilder("RightUpperLeg_IMU"));
        m_nameIMUs.Add(new StringBuilder("RightLowerLeg_IMU"));
        m_nameIMUs.Add(new StringBuilder("RightUpperArm_IMU"));
        m_nameIMUs.Add(new StringBuilder("LeftUpperArm_IMU"));

        //Name of the segment associated with the IMUs
        m_parentIMUs.Add(new StringBuilder("Pelvis"));
        m_parentIMUs.Add(new StringBuilder("Head"));
        m_parentIMUs.Add(new StringBuilder("RightThigh"));
        m_parentIMUs.Add(new StringBuilder("RightLeg"));
        m_parentIMUs.Add(new StringBuilder("RightArm"));
        m_parentIMUs.Add(new StringBuilder("LeftArm"));

        //Index of the parent
        m_parentIMUsIdx.Add(0);
        m_parentIMUsIdx.Add(2);
        m_parentIMUsIdx.Add(3);
        m_parentIMUsIdx.Add(4);
        m_parentIMUsIdx.Add(9);
        m_parentIMUsIdx.Add(12);

        //idex of the marker
        m_positionIMUs.Add(0);
        m_positionIMUs.Add(1);
        m_positionIMUs.Add(2);
        m_positionIMUs.Add(3);
        m_positionIMUs.Add(4);
        m_positionIMUs.Add(5);
    }


    public bool isLoaded(){
		return m_isloaded;
	}

    public void createModelFromStaticXsens(List<XsMatrix[]> statiqueTrial, string pathToModel, string pathToTemplate)
    {
        //string pathToModel = "C:\\Users\\ib93672\\Desktop\\Roxanne\\GIT_AcroVR\\Model\\Model.s2mMod";
        //string pathToTemplate = "C:\\Users\\ib93672\\Desktop\\Roxanne\\GIT_AcroVR\\Model\\Template\\Somersault_IMUs.s2mMod";

        //List<XsMatrix[]> statiqueTrial = scriptData.getData();
        if (statiqueTrial == null)
            print("null");
        //List<XsMatrix[]> statiqueTrial = BonesXsensInterface.dataXsens.getData();
        // Trouver le nom et path du statique à générer
        StringBuilder path = new StringBuilder(pathToModel);

        // Load du modèle générique
        string templatePath = pathToTemplate;
        if (!System.IO.File.Exists(templatePath))
        {
            Debug.Log("Template not found");
            return;
        }

        IntPtr modelTemplate = c_s2mMusculoSkeletalModel(new StringBuilder(templatePath));
        //IntPtr modelTemplate = c_biorbdModel(new StringBuilder(pathToTemplate));


        //// moyenner les centrales

        List<double[]> allIMUsMean = getIMUmean(modelTemplate, statiqueTrial);
        //  print("get imu mean");


        //// Trouver l'orientation du tronc (en comparant l'axe vertical de la première centrale avec celle du tronc)
        List<double[]> allIMUsMeanReoriented = reorientIMUtoBodyVerticalAxisXsens(modelTemplate, allIMUsMean);
      //  print("reorient to vertical axis");

        //// Remettre les IMUs dans le repère local par segment
        List<double[]> allIMUsInLocal = computeIMUsInLocal(modelTemplate, allIMUsMeanReoriented);
        //   print("compute imu local");


        //// Include translation to IMU
        List<double[]> allIMUsInLocalWithTrans = addLocalTags(modelTemplate, allIMUsInLocal);
     //   print("addLocalTags");

        //// Ajouter les imus dans le modèle template
        addIMUtoModel(modelTemplate, allIMUsInLocalWithTrans);
        //print("modelTemplate");

        //// Générer le s2mMod
         c_writeS2mMusculoSkeletalModel(modelTemplate, path);
        //print("c_writeS2mMusculoSkeletalModel");

        //c_writeBiorbdModel(modelTemplate, path);
        // print("gener fichier biorbd");
        calibre = true;


        // Finaliser
        m_isStaticDone = true;
    }
    List<double[]> reorientIMUtoBodyVerticalAxisXsens(IntPtr model, List<double[]> allIMUs)
    {
        // Recueillir les JCS à la position 0 du pelvis
        int nQTemplate = c_nQ(model);
        double[] Q = new double[nQTemplate];
        for (int i = 0; i < nQTemplate; ++i)
        {
            Q[i] = 0;

        }

        IntPtr ptr_Q = Marshal.AllocCoTaskMem(sizeof(double) * nQTemplate);
        Marshal.Copy(Q, 0, ptr_Q, nQTemplate);
        IntPtr ptrJCS = Marshal.AllocCoTaskMem(sizeof(double) * 4 * 4 * 15); //15= nombre de segment au total dans le model biorbd != nombre total d'imu
        c_globalJCS(model, ptr_Q, ptrJCS);
        double[] pelvisJCS = new double[16];
        Marshal.Copy(ptrJCS, pelvisJCS, 0, 16);
        Marshal.FreeCoTaskMem(ptr_Q);        // Récupérer le imu pelvis
        double[] pelvisIMU = allIMUs[0];        // Trouver quel axe est à peu près déjà aligné avec l'axe vertical (colonne 3) du pelvis
        double max = -1; // valeur maximale du dot product
        int idxMax = -1;
        for (int i = 0; i < 3; ++i)
        {
            double[] v1 = new double[3];
            Array.Copy(pelvisIMU, i * 4, v1, 0, 3);
            double[] v2 = new double[3];
            Array.Copy(pelvisJCS, 8, v2, 0, 3); double dotProd = Math.Abs(dot(v1, v2));
            if (dotProd > max)
            {
                max = dotProd;
                idxMax = i;
            }
        }
        // Faire l'optimisation pour aligner les axes
        // ptrJCS est trop grand (tous les segments) mais comme on s'intéresse au premier, c'est correct
        IntPtr ptrPelvisIMU = Marshal.AllocCoTaskMem(sizeof(double) * 4 * 4);
        Marshal.Copy(pelvisIMU, 0, ptrPelvisIMU, 16);
        IntPtr ptrMatRot = Marshal.AllocCoTaskMem(sizeof(double) * 4 * 4);
        c_alignSpecificAxisWithParentVertical(ptrJCS, ptrPelvisIMU, idxMax, ptrMatRot);
        Marshal.FreeCoTaskMem(ptrPelvisIMU);
        Marshal.FreeCoTaskMem(ptrJCS);        // Appliquer la rotation à chaque segment
        List<double[]> allIMUsRotated = new List<double[]>();
        IntPtr ptrM2 = Marshal.AllocCoTaskMem(sizeof(double) * 4 * 4);
        IntPtr ptr_newRotation = Marshal.AllocCoTaskMem(sizeof(double) * 4 * 4);
        foreach (double[] toRotate in allIMUs)
        {
            Marshal.Copy(toRotate, 0, ptrM2, 16);
            c_matrixMultiplication(ptrMatRot, ptrM2, ptr_newRotation);
            double[] newOrientation = new double[16];
            Marshal.Copy(ptr_newRotation, newOrientation, 0, 16);
            allIMUsRotated.Add(newOrientation);
        }
        Marshal.FreeCoTaskMem(ptrM2);
        Marshal.FreeCoTaskMem(ptr_newRotation);
        Marshal.FreeCoTaskMem(ptrMatRot);        // Retourner l'orientation tournée
        return allIMUsRotated;

    }
    // Use this for initialization
    public void LoadModel(StringBuilder pathToModel)
	{
		// Si un modèle est déjà loadé, le fermer et ouvrir le nouveau
		if (m_isloaded)
			OnDestroy();

		if (pathToModel.ToString ().CompareTo ("") == 0)
			return;

		// Load et préparation du modèle musculoskelettique
		m_s2m = c_s2mMusculoSkeletalModel(pathToModel);
		m_nQ = c_nQ (m_s2m);
		m_nQdot = c_nQDot (m_s2m);
		m_nQddot = c_nQDDot (m_s2m);
		m_nTags = c_nTags (m_s2m);

        //print("nombre de dll= " + m_nQ);
		// idem pour Kalman
		// Initial Guess pour le filtre de kalman
		IntPtr QinitialGuess = Marshal.AllocCoTaskMem(sizeof(double)*m_nQ);
		double[] Q = new double[m_nQ];
		Marshal.Copy(Q, 0, QinitialGuess, m_nQ);
		// Préparer le filtre
		m_kalman = c_s2mKalmanReconsIMU(m_s2m, QinitialGuess, XSensInterface.frameRate());
		m_nIMUs = c_nIMUs (m_s2m);
        //print("Nombre d'IMU a connecter= " + m_nIMUs);
		Marshal.FreeCoTaskMem (QinitialGuess);

		// Allouer les pointeurs des coordonnées généralisées
		m_Q = Marshal.AllocCoTaskMem(sizeof(double)*m_nQ);
		m_Qdot = Marshal.AllocCoTaskMem(sizeof(double)*m_nQdot);
		m_Qddot = Marshal.AllocCoTaskMem(sizeof(double)*m_nQddot);

		// Allouer le pointeur sur les marqueurs
		m_mark = Marshal.AllocCoTaskMem(sizeof(double)*3*m_nTags); // matrice XYZ * nMark

		// Allouer le pointeur sur les centrales
		m_IMU = Marshal.AllocCoTaskMem(sizeof(double)*3*3*m_nIMUs); // matrice 3*3*nIMU

		// Finaliser
		m_isloaded = true;
	}

	public void createModelFromStatic(string pathToModel, string pathToTemplate, List<XsMatrix[]> statiqueTrial){
		// Trouver le nom et path du statique à générer
		StringBuilder path = new StringBuilder(pathToModel);

		// Load du modèle générique
		string templatePath = pathToTemplate;
		if (!System.IO.File.Exists (templatePath)) {
			Debug.Log ("Template not found");
			return;
		}
		IntPtr modelTemplate = c_s2mMusculoSkeletalModel(new StringBuilder(templatePath));

		// moyenner les centrales
		List<double[]> allIMUsMean = getIMUmean (modelTemplate, statiqueTrial);

		// Trouver l'orientation du tronc (en comparant l'axe vertical de la première centrale avec celle du tronc)
		List<double[]> allIMUsMeanReoriented = reorientIMUtoBodyVerticalAxis(modelTemplate, allIMUsMean);

		// Remettre les IMUs dans le repère local par segment
		List<double[]> allIMUsInLocal = computeIMUsInLocal(modelTemplate, allIMUsMeanReoriented);

		// Include translation to IMU
		List<double[]> allIMUsInLocalWithTrans = addLocalTags(modelTemplate, allIMUsInLocal);

		// Ajouter les imus dans le modèle template
		addIMUtoModel(modelTemplate, allIMUsInLocalWithTrans);

		// Générer le s2mMod
		c_writeS2mMusculoSkeletalModel(modelTemplate, path);

		// Copier les dossiers de bones
		CopyFilesRecursively(new System.IO.DirectoryInfo(Path.GetDirectoryName(pathToTemplate) + "/bones"), new System.IO.DirectoryInfo(Path.GetDirectoryName(pathToModel) + "/bones"));

		// Finaliser
		m_isStaticDone = true;
	}

	public void CopyFilesRecursively(DirectoryInfo source, DirectoryInfo target) {
		target.Create ();
		foreach (DirectoryInfo dir in source.GetDirectories())
			CopyFilesRecursively(dir, target.CreateSubdirectory(dir.Name));
		foreach (FileInfo file in source.GetFiles()) {
			if (!File.Exists(Path.Combine (target.FullName, file.Name)))
				file.CopyTo (Path.Combine (target.FullName, file.Name));
		}
	}

	void addIMUtoModel(IntPtr model, List<double[]> allIMUs){
		int idxIMU = 0; 
		IntPtr imuRT = Marshal.AllocCoTaskMem(sizeof(double)*16); 
		foreach (double[] imu in allIMUs) {
			// stocker ou recevoir des matrices d'orientation des centrales inertielles
			Marshal.Copy(imu, 0, imuRT, 16);

			// Ajouter le IMU dans le modèle
			c_addIMU(model, imuRT, m_nameIMUs[idxIMU], m_parentIMUs[idxIMU], true, true);

			// Finaliser la boucle
			idxIMU += 1; 
		}
		// Libérer la mémoire qu'il faut libérer
		Marshal.FreeCoTaskMem (imuRT);
	}

	List<double[]> getIMUmean (IntPtr modelTemplate, List<XsMatrix[]> statiqueTrial){
		// Get Tags in local coordinates
		int nTagsTemplate = c_nTags (modelTemplate);
		double[] tagsInLocal = new double[3*nTagsTemplate];
		IntPtr ptrTagsInLocal = Marshal.AllocCoTaskMem (sizeof(double) * 3*nTagsTemplate);
		c_TagsInLocal (modelTemplate, ptrTagsInLocal);
		Marshal.Copy(ptrTagsInLocal, tagsInLocal, 0, 3*nTagsTemplate);
		Marshal.FreeCoTaskMem (ptrTagsInLocal);
        //print(statiqueTrial.Count);
		// Faire la moyenne du statique
		List<double[]> allIMUsMean = new List<double[]>();
		for (int j = 0; j < nbImuToConnect; ++j) { // Pour tous les IMUs
			double[] oneIMUoverTime = new double[16 * statiqueTrial.Count];
			for (int t = 0; t<statiqueTrial.Count; ++t) { // Pour tous les instants
				uint cmpElement = 0;
				// Copier la partie rotation
				for (int i = 0; i < 12; ++i) { // Ne pas process la 4ième colonne
                    if (i % 4 != 3) { // Si on est pas à la dernière ligne

                        oneIMUoverTime [t * 16 + i] = statiqueTrial[t][j].value (cmpElement % 3, cmpElement / 3); // Dispatch
                        cmpElement += 1;
					} else {
						oneIMUoverTime [t * 16 + i] = 0;
					}
				}
				// Copier la partie translation
				for (int i =0; i<3; ++i){
					oneIMUoverTime [t * 16 + 12 + i] = tagsInLocal [3 * m_positionIMUs[j] + i];
				}

				// Dernier élément est un 1
				oneIMUoverTime [16*t+15] = 1;
			}
			// Faire la moyenne
			IntPtr ptrIMU = Marshal.AllocCoTaskMem (sizeof(double) * 16 * statiqueTrial.Count);
			IntPtr ptrIMU_mean = Marshal.AllocCoTaskMem (sizeof(double) * 16);
			Marshal.Copy(oneIMUoverTime, 0, ptrIMU, 16 * statiqueTrial.Count);
			c_meanIMU(ptrIMU, statiqueTrial.Count, ptrIMU_mean);

			// Dispatch de la moyenne
			double[] imuMean = new double[16];
			Marshal.Copy(ptrIMU_mean, imuMean, 0, 16);
			allIMUsMean.Add (imuMean);
			Marshal.FreeCoTaskMem (ptrIMU);
			Marshal.FreeCoTaskMem (ptrIMU_mean);
		}
		return allIMUsMean;
	}

	List<double[]> addLocalTags (IntPtr modelTemplate, List<double[]> allIMUsInLocal){
		// Get Tags in local coordinates
		int nTagsTemplate = c_nTags (modelTemplate);
		double[] tagsInLocal = new double[3*nTagsTemplate];
		IntPtr ptrTagsInLocal = Marshal.AllocCoTaskMem (sizeof(double) * 3*nTagsTemplate);
		c_TagsInLocal (modelTemplate, ptrTagsInLocal);
		Marshal.Copy(ptrTagsInLocal, tagsInLocal, 0, 3*nTagsTemplate);

		// Faire la moyenne du statique
		List<double[]> imuOut = new List<double[]>();
		for (int j = 0; j < allIMUsInLocal.Count; ++j) { // Pour tous les IMUs
			double[] imu = new double[16];
			for (int i = 0; i<16; ++i) { // Pour tous les instants
				if (i < 12) // Copier la rotation
					imu [i] = allIMUsInLocal [j] [i];
				else if (i < 15) // Copier la translation
					imu[i] = tagsInLocal [3 * m_positionIMUs[j] + i-12];
				else // Dernier élément est un 1
					imu[i] = 1;
			}
			imuOut.Add (imu);
		}
		return imuOut;
	}

	List<double[]> computeIMUsInLocal(IntPtr model, List<double[]> imus){
		// Recueillir les JCS à la position 0
		int nQTemplate = c_nQ(model);
		double[] Q = new double[nQTemplate];
		for (int i = 0; i < nQTemplate; ++i) {
			Q [i] = 0;
		}
		IntPtr ptr_Q = Marshal.AllocCoTaskMem(sizeof(double)*nQTemplate);
		Marshal.Copy(Q, 0, ptr_Q, nQTemplate);
		IntPtr ptrJCS = Marshal.AllocCoTaskMem (sizeof(double) * 4 * 4 * 15);
		c_globalJCS(model, ptr_Q, ptrJCS); 
		double[] JCS = new double[4 * 4 * 15];
		Marshal.Copy(ptrJCS, JCS, 0, 4 * 4 * 15);
		Marshal.FreeCoTaskMem (ptr_Q);
		Marshal.FreeCoTaskMem (ptrJCS);

		// Variable de sortie
		List<double[]> outIMU = new List<double[]>();

		// Projeter dans les repères locaux
		IntPtr ptr_imuToRebase = Marshal.AllocCoTaskMem (sizeof(double) * 4 * 4);
		IntPtr ptr_parentSeg = Marshal.AllocCoTaskMem (sizeof(double) * 4 * 4);
		IntPtr ptr_rotatedIMU = Marshal.AllocCoTaskMem (sizeof(double) * 4 * 4);
		for (int i = 0; i<imus.Count; ++i){
			Marshal.Copy(imus[i], 0, ptr_imuToRebase, 16);
			Marshal.Copy(JCS, m_parentIMUsIdx[i]*16, ptr_parentSeg, 16);
			double[] test = new double[16];
			Marshal.Copy(ptr_imuToRebase, test, 0, 16);
			c_projectJCSinParentBaseCoordinate(ptr_parentSeg, ptr_imuToRebase, ptr_rotatedIMU);
			double[] rotatedIMU = new double[16];
			Marshal.Copy(ptr_rotatedIMU, rotatedIMU, 0, 16);
			outIMU.Add(rotatedIMU);
		}
		Marshal.FreeCoTaskMem (ptr_rotatedIMU);
		Marshal.FreeCoTaskMem (ptr_imuToRebase);
		Marshal.FreeCoTaskMem(ptr_parentSeg);

		return outIMU;
	}


	List<double[]> reorientIMUtoBodyVerticalAxis(IntPtr model, List<double[]> allIMUs){
		// Recueillir les JCS à la position 0 du thorax
		int nQTemplate = c_nQ(model);
		double[] Q = new double[nQTemplate];
		for (int i = 0; i < nQTemplate; ++i) {
			Q [i] = 0;
		}
		IntPtr ptr_Q = Marshal.AllocCoTaskMem(sizeof(double)*nQTemplate);
		Marshal.Copy(Q, 0, ptr_Q, nQTemplate);
		IntPtr ptrJCS = Marshal.AllocCoTaskMem (sizeof(double) * 4 * 4 * nbImuToConnect);
		c_globalJCS(model, ptr_Q, ptrJCS); 
		double[] thoraxJCS = new double[16];
		Marshal.Copy(ptrJCS, thoraxJCS, 0, 16);
		Marshal.FreeCoTaskMem (ptr_Q);

		// Récupérer le imu thorax
		double[] thoraxIMU = allIMUs [0];

		// Trouver quel axe est à peu près déjà aligné avec l'axe vertical (colonne 3) du thorax
		double max = -1; // valeur maximale du dot product
		int idxMax = -1;
		for (int i = 0; i < 3; ++i) {
			double[] v1 = new double[3];
			Array.Copy(thoraxIMU, i*4, v1, 0, 3);
			double[] v2 = new double[3];
			Array.Copy(thoraxJCS, 8, v2, 0, 3);

			double dotProd = Math.Abs (dot (v1, v2));
			if (dotProd > max) {
				max = dotProd;
				idxMax = i;
			}
		}

		// Faire l'optimisation pour aligner les axes
		// ptrJCS est trop grand (tous les segments) mais comme on s'intéresse au premier, c'est correct
		IntPtr ptrThoraxIMU = Marshal.AllocCoTaskMem (sizeof(double) * 4 * 4);
		Marshal.Copy(thoraxIMU, 0, ptrThoraxIMU, 16);
		IntPtr ptrMatRot = Marshal.AllocCoTaskMem (sizeof(double) * 4 * 4);
		c_alignSpecificAxisWithParentVertical(ptrJCS, ptrThoraxIMU, idxMax, ptrMatRot);
		Marshal.FreeCoTaskMem (ptrThoraxIMU);
		Marshal.FreeCoTaskMem(ptrJCS);

		// Appliquer la rotation à chaque segment
		List<double[]> allIMUsRotated = new List<double[]>();
		IntPtr ptrM2 = Marshal.AllocCoTaskMem (sizeof(double) * 4 * 4);
		IntPtr ptr_newRotation = Marshal.AllocCoTaskMem (sizeof(double) * 4 * 4);
		foreach (double[] toRotate in allIMUs) {
			Marshal.Copy(toRotate, 0, ptrM2, 16);
			c_matrixMultiplication (ptrMatRot, ptrM2, ptr_newRotation);
			double[] newOrientation = new double[16];
			Marshal.Copy(ptr_newRotation, newOrientation, 0, 16);
			allIMUsRotated.Add (newOrientation);
		}
		Marshal.FreeCoTaskMem (ptrM2);
		Marshal.FreeCoTaskMem (ptr_newRotation);
		Marshal.FreeCoTaskMem (ptrMatRot);

		// Retourner l'orientation tournée
		return allIMUsRotated;

	}



	/// <summary>
	/// Dot product of the specified v1 and v2.
	/// </summary>
	/// <param name="v1">V1.</param>
	/// <param name="v2">V2.</param>
	double dot(double[] v1, double[] v2){
		return v1 [0] * v2 [0] + v1 [1] * v2 [1] + v1 [2] * v2 [2];
	}

	public void OnDestroy()
	{
		// Libérer la mémoire sur les pointeurs unmanaged
		Marshal.FreeCoTaskMem (m_Q);
		Marshal.FreeCoTaskMem (m_Qdot);
		Marshal.FreeCoTaskMem (m_Qddot);

		Marshal.FreeCoTaskMem (m_mark);

		Marshal.FreeCoTaskMem (m_IMU);

		// Libérer la mémoire des fonctions c
		if (m_isloaded) {
			c_deleteS2mKalmanReconsIMU (m_kalman);
			c_deleteS2mMusculoSkeletalModel (m_s2m);
		}
	}

	public double[] KalmanReconsIMU (XsMatrix[] IMUs)
	{
		// Préparer les données d'entrée (orientation des IMUs)
		double[] imu = new double[9*m_nIMUs]; // Matrice 3*3*nIMUs
        for (int i = 0; i < m_nIMUs; ++i)
            for (uint row = 0; row < 3; ++row)
                for (uint col = 0; col < 3; ++col)
                    if (IMUs[i] == null)
                    {
                       // print("null");
                        imu[9 * i + 3 * row + col] = 0;

                    }
                    else
                    {
                       // print("full");
                        imu[9 * i + 3 * row + col] = IMUs[i].value(col, row);
                    }
		Marshal.Copy(imu, 0, m_IMU, 9*m_nIMUs);

		// Appeler la fonction de reconstruction
		c_s2mKalmanReconsIMUstep(m_s2m, m_kalman, m_IMU, m_Q, m_Qdot, m_Qddot);

		// Récupérer les données de sortie
		Q = new double[m_nQ];
        Qdot = new double[m_nQdot];
        Qddot = new double[m_nQddot];
		Marshal.Copy(m_Q, Q, 0, m_nQ);
        Marshal.Copy(m_Qdot, Qdot, 0, m_nQdot);
        Marshal.Copy(m_Qddot, Qddot, 0, m_nQddot);
        return Q;
	}

    public double[] getQ()
    {
        return Q;
    }
    public double[] getQdot()
    {
        return Qdot;
    }

    public double[] getQddot()
    {
        return Qddot;
    }
    public double[] Tags()
	{
		// Préparer les données d'entrée (coordonnées généralisées)
		double[] Q = new double[m_nQ];
		for (int i = 0; i < m_nQ; ++i) {
			Q [i] = 0;
		}
		Marshal.Copy(Q, 0, m_Q, m_nQ);

		// Appel de la fonction Tags
		c_Tags(m_s2m, m_Q, m_mark);

		// Parsing du résultat
		double[] mark = new double[3 * m_nTags];
		Marshal.Copy(m_mark, mark, 0, 3*m_nTags);    // Copy result to array.

		return mark;
	}

	// Accessor
	public bool isStaticDone(){
		return m_isStaticDone;
	}

	public double rsh(){
		// m_Q est un pointeur qu'on a utilisé lors du dernier appel de Kalman, il possède donc le dernier Q en liste
		double[] QNoScap = new double[m_nQ];
		Marshal.Copy(m_Q, QNoScap, 0, m_nQ);
		for (int i = 6; i<9; ++i) // Retirer la scapula
			QNoScap [i] = 0; 
		IntPtr ptrQNoScap = Marshal.AllocCoTaskMem (sizeof(double) * m_nQ);
		Marshal.Copy(QNoScap, 0, ptrQNoScap, m_nQ);

		// Trouver GL quand scap est là et quand il n'y est pas
		double[] allJCS = new double[16*3];
		double[] allJCSnoScap = new double[16*3];
		IntPtr ptrJCS = Marshal.AllocCoTaskMem (sizeof(double) * 4 * 4 * nbImuToConnect);
		c_globalJCS (m_s2m, m_Q, ptrJCS);
		Marshal.Copy(ptrJCS, allJCS,  0, 16*3); // Thorax est le 1er segment, Scapula 2ième, Humerus 3ième
		c_globalJCS (m_s2m, ptrQNoScap, ptrJCS);
		Marshal.Copy(ptrJCS, allJCSnoScap,  0, 16*3); // Thorax est le 1er segment, Scapula 2ième, Humerus 3ième
		Marshal.FreeCoTaskMem (ptrQNoScap);

		// Calculer TH
		double[] th_mat = new double[16];
		double[] thNoScap_mat = new double[16];
		IntPtr ptrThoraxJCS = Marshal.AllocCoTaskMem (sizeof(double) * 4 * 4);
		IntPtr ptrHumJCS = Marshal.AllocCoTaskMem (sizeof(double) * 4 * 4);
		Marshal.Copy(allJCS, 16*0, ptrThoraxJCS, 16); // 0 est le thorax
		Marshal.Copy(allJCS, 16*2, ptrHumJCS, 16); // 2 est l'humérus
		c_projectJCSinParentBaseCoordinate(ptrThoraxJCS, ptrHumJCS, ptrJCS);
		Marshal.Copy(ptrJCS, th_mat,  0, 16); 
		Marshal.Copy(allJCSnoScap, 16*2, ptrHumJCS, 16); // 2 est l'humérus
		c_projectJCSinParentBaseCoordinate(ptrThoraxJCS, ptrHumJCS, ptrJCS);
		Marshal.Copy(ptrJCS, thNoScap_mat,  0, 16); 
		Marshal.FreeCoTaskMem (ptrJCS);
		Marshal.FreeCoTaskMem (ptrHumJCS);
		Marshal.FreeCoTaskMem (ptrThoraxJCS);

		double th = extractElevationFromMatriceRotation (th_mat);
		double th_noScap = extractElevationFromMatriceRotation (thNoScap_mat);

		double rsh = (th-th_noScap) / th_noScap;
		return rsh;

	}

	/// <summary>
	/// Extracts the elevation angle (Y) from matrix of rotation assuming ZYZ sequence.
	/// </summary>
	/// <returns>The elevation angle from matrix of rotation.</returns>
	/// <param name="matrice">Matrix of rotation.</param>
	double extractElevationFromMatriceRotation(double[] matrice){
		// Arccos de l'élément 3,3 de la matrice de rotation est l'élévation dans une séquence zyz
		double[] angles = new double[3];
		IntPtr ptrAngles = Marshal.AllocCoTaskMem (sizeof(double) * 3);

		// Appliquer la rotation à chaque segment
		IntPtr ptrMatrice = Marshal.AllocCoTaskMem (sizeof(double) * 4 * 4);
		Marshal.Copy(matrice, 0, ptrMatrice, 16);
		StringBuilder sequence = new StringBuilder("zyz");

		// Appeler la fonction principale
		c_transformMatrixToCardan(ptrMatrice, sequence, ptrAngles);

		Marshal.Copy(ptrAngles, angles,  0, 3);

		return angles [2];
	}

	public double[] getCardanTransformationFromParentToSegment(int segIdx){
		double[] rtInCard = new double[6]; // 3 translations, 3 rotations

		// Local coordinate system
		double[] lcs = new double[16];
		IntPtr ptrLcs = Marshal.AllocCoTaskMem (sizeof(double) * 16);
		c_localJCS (m_s2m, segIdx, ptrLcs);
		Marshal.Copy(ptrLcs, lcs,  0, 16); 
		rtInCard [0] = lcs [12]; // Prendre les translations
		rtInCard [1] = lcs [13];
		rtInCard [2] = lcs [14];

		// Convertir en angle de cardan
		double[] cardan = new double[3];
		IntPtr ptrCardanOut = Marshal.AllocCoTaskMem (sizeof(double) * 3);
		c_transformMatrixToCardan (ptrLcs, new StringBuilder("xyz"), ptrCardanOut);
		Marshal.Copy(ptrCardanOut, cardan,  0, 3); 
		rtInCard [3] = cardan [0]; // Prendre les rotations
		rtInCard [4] = cardan [1];
		rtInCard [5] = cardan [2];

		// Retourner la réponse
		return rtInCard; // Transfomer en système main gauche
	}

}
