using System;
using System.Collections;
using System.Collections.Generic;
using System.ComponentModel;
using System.Linq;
using UnityEngine;
using UnityEngine.UI;
using Xsens;
using XDA;

public class XSensInterface : MonoBehaviour
{
	private bool m_debugWithoutXSensMode = false;
	public double[] m_imuSimulated = new double[3*S2M_model.nbImuToConnect];//[3,S2M_model.nbImuToConnect];

	private MyXda _xda;
	private XsDevice _measuringDevice = null;
	private Dictionary<XsDevice, MyMtCallback> _measuringMts = new Dictionary<XsDevice, MyMtCallback>();
	private Dictionary<uint, ConnectedMtData> _connectedMtwData = new Dictionary<uint, ConnectedMtData>();

	private List<MasterInfo> _allScannedStations = new List<MasterInfo>();
	private MasterInfo _selectedScanStation;

	public GameObject _PanelConnexion;
	private int _numberIMUtoConnect = S2M_model.nbImuToConnect;
	private bool _canUseTheSystem = false; // True si la connexion est active et correcte
	private bool _hasPrepared = false;

	static public int frameRate(){
		return 50; // ON NE PEUT CHANGER CETTE VALEUR.. obtenue expérimentalement selon FixedUpdate
	}

	void Start()
	{
		if (m_debugWithoutXSensMode)
			return;
		_xda = new MyXda();
	}

	// Accessors
	public bool canUseTheSystem(){
		return _canUseTheSystem;
	}
	public int numberIMUtoConnect(){
		return _numberIMUtoConnect;
	}

	public XsMatrix[] dataMatrix(){
		XsMatrix[] IMUs = new XsMatrix[numberIMUtoConnect()];
		int cmp = 0;
		foreach (KeyValuePair<uint, ConnectedMtData> data in _connectedMtwData) {
			if (m_debugWithoutXSensMode) {
				IMUs [cmp] = new XsMatrix(new XsQuaternion(new XsEuler (m_imuSimulated [data.Key * 3 + 0], m_imuSimulated [data.Key * 3 + 1], m_imuSimulated [data.Key * 3 + 2])));
			} else {
				if (data.Value._orientationMatrix != null) {
					IMUs [cmp] = data.Value._orientationMatrix; // Attention, je ne sais pas si foreach met toujours dans le même ordre les choses
				}
			}
			cmp += 1;
		}
		return IMUs;
	}
	public XsEuler[] dataEuler(){
		XsEuler[] IMUs = new XsEuler[numberIMUtoConnect()];
		int cmp = 0;
		foreach (KeyValuePair<uint, ConnectedMtData> data in _connectedMtwData) {
			if (m_debugWithoutXSensMode) {
				IMUs [cmp] = new XsEuler (m_imuSimulated [data.Key * 3 + 0], m_imuSimulated [data.Key * 3 + 1], m_imuSimulated [data.Key * 3 + 2]);
			} else {
				if (data.Value._orientation != null) {
					IMUs [cmp] = data.Value._orientation; // Attention, je ne sais pas si foreach met toujours dans le même ordre les choses
				}
			}
			cmp += 1;
		}
		return IMUs;
	}

	public void prepareSystem(){
		if (_hasPrepared) // S'assurer de ne le faire qu'une fois
			return;

		// Trouver le dock sur lequel se brancher
		if (!scan()) { 
			Debug.Log("No station was found, make sure the drivers were installed correctly.");
			return;
		}

		// Activer le signal radio
		if (!enable()){
			Debug.Log("There was an unexpected failure enabling the station.");
			return;
		}

		// Démarrer la connexion avec les centrales
		_hasPrepared = true;
		StartCoroutine(measure());
	}

	void OnDestroy()
	{
		// Inutile, mais c'est pour s'en rappeler si on déplacer la fermeture
		_hasPrepared = false;
		_canUseTheSystem = false;

		if (_measuringDevice != null) {
			if (_measuringDevice.isRecording ()) {
				_measuringDevice.stopRecording ();
			}
			_measuringDevice.gotoConfig ();
			_measuringDevice.disableRadio();
			_measuringDevice.clearCallbackHandlers ();
		}

		_measuringMts.Clear();

		if (_xda != null) {
			_xda.Dispose ();
			_xda = null;
		}
	}

	protected bool scan()
	{
		if (m_debugWithoutXSensMode) {
			return true;
		}

		if (_hasPrepared)
			return true;

		_xda.scanPorts();
		if (_xda._DetectedDevices.Count > 0)
		{
			foreach (XsPortInfo portInfo in _xda._DetectedDevices)
			{
				if (portInfo.deviceId().isWirelessMaster())
				{
					_xda.openPort(portInfo);
					MasterInfo ai = new MasterInfo(portInfo.deviceId());
					ai.ComPort = portInfo.portName();
					ai.BaudRate = portInfo.baudrate();
					_allScannedStations.Add(ai);
					break;
				}
			}

			if (_allScannedStations.Count > 0)
			{ 
				// Sélectionner le premier par défaut (obligatoirement)
				_selectedScanStation = _allScannedStations [0];
				return true;
			}
		}
		return false; // Si on se rend ici, c'est qu'on a trouvé aucun dock
	}


	protected bool enable()
	{
		if (m_debugWithoutXSensMode) {
			return true;
		}

		if (_selectedScanStation != null) {
			XsDevice device = _xda.getDevice (_selectedScanStation.DeviceId);
			if (device.isRadioEnabled ())
				device.disableRadio ();

			// Tester tous les channels radio jusqu'à ce qu'un fonctionne
			for (int i = 12; i < 26; ++i) {
				if (device.enableRadio (i)) {
					return true;
				}
			}
		}
		return false; // Si on arrive ici, c'est qu'aucun ne fonctionne
	}

	IEnumerator measure()
	{
		if (m_debugWithoutXSensMode) {
			_connectedMtwData.Clear();
			ConnectedMtData mtwData = new ConnectedMtData();
			for (uint i = 0; i < numberIMUtoConnect(); ++i)
				_connectedMtwData.Add (i, mtwData);
			_canUseTheSystem = true;
			yield break;
		}

		_connectedMtwData.Clear();
		_measuringDevice = _xda.getDevice(_selectedScanStation.DeviceId);
		_measuringDevice.setUpdateRate (frameRate());

		// Faire une boucle jusqu'à ce que toutes les centrales sont connectées
		_PanelConnexion.SetActive(true);
		Text textToUpdate = _PanelConnexion.GetComponentInChildren<Text> ();

		XsDevicePtrArray deviceIds;
		string stringToUse = textToUpdate.text;
		do {
			deviceIds = _measuringDevice.children();
			textToUpdate.text = string.Format(stringToUse, deviceIds.size (), _numberIMUtoConnect);
			yield return 0;
		} while (deviceIds.size () != _numberIMUtoConnect);
		_measuringDevice.setOptions(XsOption.XSO_Orientation, XsOption.XSO_None);
		_measuringDevice.gotoMeasurement();
		_PanelConnexion.SetActive(false);

		for (uint i = 0; i < deviceIds.size(); i++)
		{
			XsDevice mtw = new XsDevice(deviceIds.at(i));
			MyMtCallback callback = new MyMtCallback();

			ConnectedMtData mtwData = new ConnectedMtData();
			_connectedMtwData.Add(mtw.deviceId().toInt(), mtwData);

			// connect signals
			callback.DataAvailable += new EventHandler<DataAvailableArgs>(_callbackHandler_DataAvailable);

			mtw.addCallbackHandler(callback);
			_measuringMts.Add(mtw, callback);
		}

		Debug.Log(string.Format("MTw's Connected: {0}", deviceIds.size()));

		// On est prêt
		_canUseTheSystem = true;
	}

	void _callbackHandler_DataAvailable(object sender, DataAvailableArgs e)
	{
		//Getting Euler angles.
		XsMatrix oriMatrix = e.Packet.orientationMatrix ();
		XsEuler oriEuler = e.Packet.orientationEuler ();
		_connectedMtwData [e.Device.deviceId ().toInt ()]._orientation = oriEuler;
		_connectedMtwData [e.Device.deviceId ().toInt ()]._orientationMatrix = oriMatrix;

	}
}




﻿