//------------------------------------------------------------------------------
// <auto-generated />
//
// This file was automatically generated by SWIG (http://www.swig.org).
// Version 3.0.5
//
// Do not make changes to this file unless you know what you are doing--modify
// the SWIG interface file instead.
//------------------------------------------------------------------------------

namespace XDA {

public class XsStringImpl : global::System.IDisposable {
  private global::System.Runtime.InteropServices.HandleRef swigCPtr;
  protected bool swigCMemOwn;

  internal XsStringImpl(global::System.IntPtr cPtr, bool cMemoryOwn) {
    swigCMemOwn = cMemoryOwn;
    swigCPtr = new global::System.Runtime.InteropServices.HandleRef(this, cPtr);
  }

  internal static global::System.Runtime.InteropServices.HandleRef getCPtr(XsStringImpl obj) {
    return (obj == null) ? new global::System.Runtime.InteropServices.HandleRef(null, global::System.IntPtr.Zero) : obj.swigCPtr;
  }

  ~XsStringImpl() {
    Dispose();
  }

  public virtual void Dispose() {
    lock(this) {
      if (swigCPtr.Handle != global::System.IntPtr.Zero) {
        if (swigCMemOwn) {
          swigCMemOwn = false;
          xsensdeviceapiPINVOKE.delete_XsStringImpl(swigCPtr);
        }
        swigCPtr = new global::System.Runtime.InteropServices.HandleRef(null, global::System.IntPtr.Zero);
      }
      global::System.GC.SuppressFinalize(this);
    }
  }

  public XsStringImpl(uint count, string src) : this(xsensdeviceapiPINVOKE.new_XsStringImpl__SWIG_0(count, src), true) {
  }

  public XsStringImpl(uint count) : this(xsensdeviceapiPINVOKE.new_XsStringImpl__SWIG_1(count), true) {
  }

  public XsStringImpl() : this(xsensdeviceapiPINVOKE.new_XsStringImpl__SWIG_2(), true) {
  }

  public XsStringImpl(XsStringImpl other) : this(xsensdeviceapiPINVOKE.new_XsStringImpl__SWIG_3(XsStringImpl.getCPtr(other)), true) {
    if (xsensdeviceapiPINVOKE.SWIGPendingException.Pending) throw xsensdeviceapiPINVOKE.SWIGPendingException.Retrieve();
  }

  public XsStringImpl(string arg0, uint sz, XsDataFlags flags) : this(xsensdeviceapiPINVOKE.new_XsStringImpl__SWIG_4(arg0, sz, (int)flags), true) {
  }

  public XsStringImpl(string arg0, uint sz) : this(xsensdeviceapiPINVOKE.new_XsStringImpl__SWIG_5(arg0, sz), true) {
  }

  public void clear() {
    xsensdeviceapiPINVOKE.XsStringImpl_clear(swigCPtr);
  }

  public void reserve(uint count) {
    xsensdeviceapiPINVOKE.XsStringImpl_reserve(swigCPtr, count);
  }

  public uint reserved() {
    uint ret = xsensdeviceapiPINVOKE.XsStringImpl_reserved(swigCPtr);
    return ret;
  }

  public SWIGTYPE_p_XsArrayDescriptor descriptor() {
    SWIGTYPE_p_XsArrayDescriptor ret = new SWIGTYPE_p_XsArrayDescriptor(xsensdeviceapiPINVOKE.XsStringImpl_descriptor(swigCPtr), false);
    return ret;
  }

  public char value(uint index) {
    char ret = xsensdeviceapiPINVOKE.XsStringImpl_value(swigCPtr, index);
    return ret;
  }

  public char first() {
    char ret = xsensdeviceapiPINVOKE.XsStringImpl_first(swigCPtr);
    return ret;
  }

  public char last() {
    char ret = xsensdeviceapiPINVOKE.XsStringImpl_last(swigCPtr);
    return ret;
  }

  public char at(uint index) {
    char ret = xsensdeviceapiPINVOKE.XsStringImpl_at__SWIG_0(swigCPtr, index);
    return ret;
  }

  public void insert(char item, uint index) {
    xsensdeviceapiPINVOKE.XsStringImpl_insert__SWIG_0(swigCPtr, item, index);
  }

  public void insert(string items, uint index, uint count) {
    xsensdeviceapiPINVOKE.XsStringImpl_insert__SWIG_1(swigCPtr, items, index, count);
  }

  public void push_back(char item) {
    xsensdeviceapiPINVOKE.XsStringImpl_push_back(swigCPtr, item);
  }

  public void pop_back(uint count) {
    xsensdeviceapiPINVOKE.XsStringImpl_pop_back__SWIG_0(swigCPtr, count);
  }

  public void pop_back() {
    xsensdeviceapiPINVOKE.XsStringImpl_pop_back__SWIG_1(swigCPtr);
  }

  public void push_front(char item) {
    xsensdeviceapiPINVOKE.XsStringImpl_push_front(swigCPtr, item);
  }

  public void pop_front(uint count) {
    xsensdeviceapiPINVOKE.XsStringImpl_pop_front__SWIG_0(swigCPtr, count);
  }

  public void pop_front() {
    xsensdeviceapiPINVOKE.XsStringImpl_pop_front__SWIG_1(swigCPtr);
  }

  public uint size() {
    uint ret = xsensdeviceapiPINVOKE.XsStringImpl_size(swigCPtr);
    return ret;
  }

  public void erase(uint index, uint count) {
    xsensdeviceapiPINVOKE.XsStringImpl_erase__SWIG_0(swigCPtr, index, count);
  }

  public void erase(uint index) {
    xsensdeviceapiPINVOKE.XsStringImpl_erase__SWIG_1(swigCPtr, index);
  }

  public void assign(uint count, string src) {
    xsensdeviceapiPINVOKE.XsStringImpl_assign(swigCPtr, count, src);
  }

  public void resize(uint count) {
    xsensdeviceapiPINVOKE.XsStringImpl_resize(swigCPtr, count);
  }

  public void setSize(uint count) {
    xsensdeviceapiPINVOKE.XsStringImpl_setSize(swigCPtr, count);
  }

  public void append(XsStringImpl other) {
    xsensdeviceapiPINVOKE.XsStringImpl_append(swigCPtr, XsStringImpl.getCPtr(other));
    if (xsensdeviceapiPINVOKE.SWIGPendingException.Pending) throw xsensdeviceapiPINVOKE.SWIGPendingException.Retrieve();
  }

  public bool empty() {
    bool ret = xsensdeviceapiPINVOKE.XsStringImpl_empty(swigCPtr);
    return ret;
  }

  public void swap(XsStringImpl other) {
    xsensdeviceapiPINVOKE.XsStringImpl_swap(swigCPtr, XsStringImpl.getCPtr(other));
    if (xsensdeviceapiPINVOKE.SWIGPendingException.Pending) throw xsensdeviceapiPINVOKE.SWIGPendingException.Retrieve();
  }

  public int find(char needle) {
    int ret = xsensdeviceapiPINVOKE.XsStringImpl_find(swigCPtr, needle);
    return ret;
  }

  public void removeDuplicates() {
    xsensdeviceapiPINVOKE.XsStringImpl_removeDuplicates(swigCPtr);
  }

  public void sort() {
    xsensdeviceapiPINVOKE.XsStringImpl_sort(swigCPtr);
  }

}

}
