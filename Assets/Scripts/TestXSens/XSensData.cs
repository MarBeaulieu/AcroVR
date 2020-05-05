using System.IO;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using XDA;

public class XSensData { 

	List<XsMatrix[]> m_data = new List<XsMatrix[]>();
	List<long> m_timeStamps = new List<long>();
	long m_nFrame = 0;
	string m_modelPath = "";

	// Constructor
	public XSensData(string modelPath){
		m_modelPath = modelPath;
	}

	public XSensData(){
		
	}

	public void setModelPath(string path){
		m_modelPath = path;
	}
	public string modelPath(){
		return m_modelPath;
	}

	public void Add(long timeStamps, XsMatrix[] data){
		m_data.Add (data);
		m_timeStamps.Add (timeStamps);
		m_nFrame += 1;
	}

	public long nFrame(){
		return m_nFrame;
	}

	public List<XsMatrix[]> getData(){
		return m_data;
	}
	public XsMatrix[] getData(int i){
		return m_data [i];
	}

	public List<long> getTimeStamps(){
		return m_timeStamps;
	}
	public long getTimeStamp(int i){
		return m_timeStamps [i];
	}

	public void Clear(){
		m_data.Clear ();
		m_timeStamps.Clear();
		m_nFrame = 0;
	}


	/// <summary>
	/// Writes the trial.
	/// </summary>
	public void writeTrialToBinary(string filepath){
		DirectoryInfo dirpath = new System.IO.DirectoryInfo(Path.GetDirectoryName(filepath));
		if (dirpath.Exists == false) {
			dirpath.Create ();

			// S'il n'existe pas encore, c'est qu'il y a une erreur, ne pas écrire rien
			dirpath = new System.IO.DirectoryInfo(Path.GetDirectoryName(filepath));
			if (dirpath.Exists == false) {
				Debug.Log ("Le dossier n'a pu être créé, le fichier ne sera pas enregistré");
				return;
			}
		}

		double version = 1.0;
		long nbFrame = nFrame();
		long nCentrale = (long)S2M_model.nbImuToConnect;
		long nElementsInMatrix = (long)9; // matrice 3x3

		// Enregistrer l'essai dans un fichier
		using (BinaryWriter writer = new BinaryWriter (File.Open (filepath, FileMode.Create))) {
			// Enregistrer des éléments pour permettre d'aider à réouvrir plus tard
			writer.Write (version);
			writer.Write (m_modelPath);
			writer.Write (nbFrame);
			writer.Write (nCentrale);
			writer.Write (nElementsInMatrix);

			for (int i = 0; i < nbFrame; ++i) { // Pour chaque frame
				writer.Write (getTimeStamp(i)); // Enregistrer le time stamp
				for (int j = 0; j < nCentrale; ++j) { // Pour chaque centrale
					for (uint k = 0; k < nElementsInMatrix; ++k) { // Pour chaque élément de la matrice de rotation
						writer.Write (getData(i) [j].value (k % 3, k / 3));
					}
				}
			}
		}
	}

	/// <summary>
	/// Reads a trial.
	/// </summary>
	public void readTrialFromBinary(string path){
		Clear ();

		// Enregistrer l'essai dans un fichier
		using (BinaryReader reader = new BinaryReader (File.Open (path, FileMode.Open))) {
			// Lire des éléments pour permettre d'avoir les dimensions
			double version = reader.ReadDouble ();
			m_modelPath = reader.ReadString ();
			long nFrame = reader.ReadInt64 ();
			long nCentrale = reader.ReadInt64 ();
			long nElementsInMatrix = reader.ReadInt64 ();

			// Lire ce que l'on sait sur l'essai
			//string nomTrial = reader.ReadString();
			//string categorie = reader.ReadString();
			//string description = reader.ReadString ();

			if (version == 1.0) {
				for (int i = 0; i < nFrame; ++i) { // Pour chaque frame
					long timeStamps = reader.ReadInt64 (); // Lire un timestamp Int64 = long
					XsMatrix[] all_mat = new XsMatrix[nCentrale];
					for (int j = 0; j < nCentrale; ++j) { // Pour chaque centrale
						XsMatrix mat = new XsMatrix (3,3);
						for (uint k = 0; k < nElementsInMatrix; ++k) { // Pour chaque élément de la matrice de rotation
							mat.setValue (k % 3, k / 3, reader.ReadDouble ());
						}
						all_mat [j] = mat;
					}
				 	Add (timeStamps, all_mat);
				}
			}
		}
	}


}
