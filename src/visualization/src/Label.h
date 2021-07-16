/*
 * Copyright (C) 2021 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#ifndef IDYNTREE_LABEL_H
#define IDYNTREE_LABEL_H

#include <iDynTree/Visualizer.h>
#include <irrlicht.h>

class Label : public iDynTree::ILabel
{
    irr::gui::IGUIFont* m_font{nullptr};
    irr::scene::IBillboardTextSceneNode* m_label{nullptr};
    float m_ratio{1.0};
    float m_width{0.0};
    float m_height{-0.1};
    std::string m_text;
    std::wstring m_wtext;
    iDynTree::ColorViz m_color{0.0, 0.0, 0.0, 1.0};

public:

    Label();

    Label(const Label& other);

    Label(Label&& other);

    Label& operator=(const Label& other);

    Label& operator=(Label&& other);

    virtual ~Label();

    void init(irr::scene::ISceneManager* smgr, irr::scene::ISceneNode* parent = nullptr);

    bool initialized() const;

    virtual void setText(const std::string& text) final;

    virtual std::string getText() const final;

    virtual void setSize(float height) final;

    virtual void setSize(float width, float height) final;

    virtual float width() const final;

    virtual float height() const final;

    virtual void setPosition(const iDynTree::Position& position) final;

    virtual iDynTree::Position getPosition() const final;

    virtual void setColor(const iDynTree::ColorViz& color) final;

    virtual void setVisible(bool visible) final;
};

#endif // LABEL_H
