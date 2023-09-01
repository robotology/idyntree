// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include "Label.h"
#include <cassert>
#include <string>
#include <codecvt>
#include <locale>
#include "IrrlichtUtils.h"


Label::Label()
{

}

Label::Label(const Label &other)
{
    operator=(other);
}

Label::Label(Label &&other)
{
    operator=(other);
}

Label &Label::operator=(const Label &other)
{
    m_font = other.m_font;
    if (m_font)
    {
        m_font->grab();
    }

    m_label = other.m_label;
    if (m_label)
    {
        m_label->grab();
    }
    return *this;
}

Label &Label::operator=(Label &&other)
{
    m_font = other.m_font;
    other.m_font = nullptr;

    m_label = other.m_label;
    other.m_label = nullptr;

    return *this;
}

Label::~Label()
{
    if (m_font)
    {
        m_font->drop();
        m_font = nullptr;
    }

    if (m_label)
    {
        m_label->drop();
        m_label = nullptr;
    }
}

void Label::init(irr::scene::ISceneManager *smgr, irr::scene::ISceneNode *parent)
{
    assert(smgr);
    m_font = smgr->getGUIEnvironment()->getBuiltInFont();
    m_font->grab();

    m_label = smgr->addBillboardTextSceneNode(m_font, L"", parent);
    m_label->grab();
}

bool Label::initialized() const
{
    return m_font && m_label;
}

void Label::setText(const std::string &text)
{
    assert(m_font);
    assert(m_label);
    using convert_typeX = std::codecvt_utf8<wchar_t>;
    std::wstring_convert<convert_typeX, wchar_t> converterX;
    m_wtext = converterX.from_bytes(text).c_str(); //See https://stackoverflow.com/a/18374698
    m_label->setText(m_wtext.c_str());
    m_text = text;
    if (m_ratio >= 0)
    {
        auto dims = m_font->getDimension(m_wtext.c_str());
        if (dims.Height)
        {
            m_ratio = dims.Width / (float) dims.Height;
            m_width = m_ratio * m_height;
            m_label->setSize({m_width, m_height});
        }
    }
    setColor(m_color);
}

std::string Label::getText() const
{
    return m_text;
}

void Label::setSize(float height)
{
    assert(m_font);
    assert(m_label);

    m_height = -height;
    auto dims = m_font->getDimension(m_wtext.c_str());
    if (dims.Height)
    {
        m_ratio = dims.Width / (float) dims.Height;
        m_width = m_ratio * m_height;
        m_label->setSize({m_width, m_height});
    }

}

void Label::setSize(float width, float height)
{
    m_height = -height;
    m_width = -width;
    m_ratio = -1.0;
    m_label->setSize({m_width, m_height});
}

float Label::width() const
{
    return -m_width;
}

float Label::height() const
{
    return -m_height;
}

void Label::setPosition(const iDynTree::Position &position)
{
    assert(m_label);
    m_label->setPosition(iDynTree::idyntree2irr_pos(position));
}

iDynTree::Position Label::getPosition() const
{
    assert(m_label);
    return iDynTree::irr2idyntree_pos(m_label->getPosition());
}

void Label::setColor(const iDynTree::ColorViz &color)
{
    assert(m_label);
    m_color = color;
    m_label->setTextColor(iDynTree::idyntree2irrlicht(color).toSColor());
    m_label->setColor(iDynTree::idyntree2irrlicht(color).toSColor());
}

void Label::setVisible(bool visible)
{
    assert(m_label);
    m_label->setVisible(visible);
    if (visible)
    {
        setText(m_text);
    }
    else
    {
        m_label->setText(L""); //This is a workaround since it seems that the visibility of labels is not considered with viewports
    }
}
