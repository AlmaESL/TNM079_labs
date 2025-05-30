#pragma once

#ifndef _GL_OBJECT
#define _GL_OBJECT

#ifdef __WXMAC__
#include "GLUT/glut.h"
#else
#include <GL/glut.h>
#endif

#include "Util/BlackWhiteColorMap.h"

#include <cstring>
#include <list>
#include <string>
#include <typeinfo>

class GLObject {
public:
    class VisualizationMode {
        friend class GLObject;

    public:
        inline size_t GetID() const { return mID; }
        inline const std::string& GetName() const { return mName; }

        bool operator==(const VisualizationMode& mode) { return mID == mode.mID; }

    protected:
        VisualizationMode(size_t ID = 0, const std::string& name = "<not set>")
            : mID(ID), mName(name) {}

        size_t mID;
        std::string mName;
    };

    GLObject() : GLObject("no-name") {}

    GLObject(const std::string& name) : mName(name) {
        mHover = mSelected = false;
        mWireframe = false;
        mShowNormals = false;
        mOpacity = 1.f;
        mAutoMinMax = true;
        mMinCMap = 0.f;
        mMaxCMap = 1.f;

        std::list<std::string> colormaps = ColorMapFactory::GetColorMaps();
        mColorMap = (colormaps.size() > 0) ? ColorMapFactory::New(colormaps.front()) : NULL;
    }

    virtual ~GLObject() {}

    virtual void Render();

    void SetName(const std::string& name) { mName = name; }
    const std::string& GetName() const { return mName; }
    virtual const char* GetTypeName() = 0;

    virtual void SetColorMap(ColorMap* colormap) { mColorMap = colormap; }
    virtual ColorMap* GetColorMap() const  { return mColorMap; }

    virtual void SetVisualizationMode(const VisualizationMode& source) {
        mVisualizationMode = source;
    }
    virtual const VisualizationMode& GetVisualizationMode() const { return mVisualizationMode; }

    virtual void SetWireframe(bool enable = true) { mWireframe = enable; }
    virtual bool GetWireframe() const  { return mWireframe; }

    virtual void SetShowNormals(bool enable = true) { mShowNormals = enable; }
    virtual bool GetShowNormals() const { return mShowNormals; }

    virtual void SetOpacity(float opacity) { mOpacity = opacity; }
    inline float GetOpacity() const { return mOpacity; }

    virtual std::list<VisualizationMode> GetVisualizationModes() {
        return std::list<VisualizationMode>();
    }

    void Hover() { mHover = true; }
    void UnHover() { mHover = false; }
    void ToggleHover() { mHover = !mHover; }
    bool IsHovering() { return mHover; }

    void Select() { mSelected = true; }
    void DeSelect() { mSelected = false; }
    void ToggleSelect() { mSelected = !mSelected; }
    bool IsSelected() const { return mSelected; }

    virtual void PickChildren(GLuint* objects, int numberOfObjects) {}

    float mMinCMap, mMaxCMap;
    bool mAutoMinMax;

protected:
    std::string mName;
    bool mHover, mSelected;
    ColorMap* mColorMap;
    VisualizationMode mVisualizationMode;
    float mOpacity;
    bool mWireframe;
    bool mShowNormals;

    void CheckGLError();

    void RenderString(float x, float y, float z, char* string) {
        glRasterPos3f(x, y, z);
        int len = (int)strlen(string);
        for (int i = 0; i < len; i++) {
            glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, string[i]);
        }
    }

    static VisualizationMode NewVisualizationMode(const std::string& name) {
        static size_t mVisualizationModeCounter = 0;
        mVisualizationModeCounter++;
        return VisualizationMode(mVisualizationModeCounter, name);
    }
};

#endif
