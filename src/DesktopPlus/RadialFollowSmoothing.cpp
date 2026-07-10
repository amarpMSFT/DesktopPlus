/* This Source Code Form is subject to the terms of the Mozilla Public
* License, v. 2.0. If a copy of the MPL was not distributed with this
* file, You can obtain one at https://mozilla.org/MPL/2.0/. */

//This source file and accompanying header is based on
//AbstractQbit's Radial Follow Smoothing OpenTabletDriver Plugin (https://github.com/AbstractQbit/AbstractOTDPlugins) (RadialFollowCore.cs only)

#include "RadialFollowSmoothing.h"

#include <cmath>

double RadialFollowCore::GetOuterRadius()
{
    return m_RadiusOuter;
}

void RadialFollowCore::SetOuterRadius(double value)
{
    m_RadiusOuter = clamp(value, 0.0, 1000000.0);
}

double RadialFollowCore::GetInnerRadius()
{
    return m_RadiusInner;
}

void RadialFollowCore::SetInnerRadius(double value)
{
    m_RadiusInner = clamp(value, 0.0, 1000000.0);
}

double RadialFollowCore::GetSmoothingCoefficient()
{
    return m_SmoothingCoef;
}

void RadialFollowCore::SetSmoothingCoefficient(double value)
{
    m_SmoothingCoef = clamp(value, 0.0001, 1.0);
}

double RadialFollowCore::GetSoftKneeScale()
{
    return m_SoftKneeScale;
}

void RadialFollowCore::SetSoftKneeScale(double value)
{
    m_SoftKneeScale = clamp(value, 0.0, 100.0);
    UpdateDerivedParams();
}

double RadialFollowCore::GetSmoothingLeakCoefficient()
{
    return m_SmoothingLeakCoef;
}

void RadialFollowCore::SetSmoothingLeakCoefficient(double value)
{
    m_SmoothingLeakCoef = clamp(value, 0.0, 1.0);
}

bool RadialFollowCore::GetDetectInterruptions()
{
    return m_DetectInterruptions;
}

void RadialFollowCore::SetDetectInterruptions(bool value)
{
    m_DetectInterruptions = value;
}

void RadialFollowCore::ApplyPresetSettings(int preset_id)
{
    preset_id = clamp(preset_id, 0, 5);

    //These are just presets used by Desktop+, in hopes that they make sense for laser pointing and are easier to use than adjusting values directly
    switch (preset_id)
    {
        case 0: //Not really used, skip calling Filter() entirely instead
        {
            SetOuterRadius(0.0);
            SetInnerRadius(0.0);
            SetSmoothingCoefficient(0.0);
            SetSoftKneeScale(0.0);
            SetSmoothingLeakCoefficient(0.0);
            break;
        }
        case 1:
        {
            SetOuterRadius(5.0);
            SetInnerRadius(0.5);
            SetSmoothingCoefficient(0.95);
            SetSoftKneeScale(1.0);
            SetSmoothingLeakCoefficient(0.0);
            break;
        }
        case 2:
        {
            SetOuterRadius(5.0);
            SetInnerRadius(3.0);
            SetSmoothingCoefficient(0.95);
            SetSoftKneeScale(1.0);
            SetSmoothingLeakCoefficient(0.0);
            break;
        }
        case 3:
        {
            SetOuterRadius(7.5);
            SetInnerRadius(4.5);
            SetSmoothingCoefficient(1.0);
            SetSoftKneeScale(5.0);
            SetSmoothingLeakCoefficient(0.25);
            break;
        }
        case 4:
        {
            SetOuterRadius(12.5);
            SetInnerRadius(7.5);
            SetSmoothingCoefficient(1.0);
            SetSoftKneeScale(10.0);
            SetSmoothingLeakCoefficient(0.5);
            break;
        }
        case 5:
        {
            SetOuterRadius(32.0);
            SetInnerRadius(12.0);
            SetSmoothingCoefficient(1.0);
            SetSoftKneeScale(50.0);
            SetSmoothingLeakCoefficient(0.75);
            break;
        }
    }
}

float RadialFollowCore::SampleRadialCurve(float dist)
{
    return (float)DeltaFn(dist, m_XOffset, m_ScaleComp, m_RadiusInner);
}

Vector2 RadialFollowCore::Filter(const Vector2& target)
{
    Vector2 direction = target - m_LastPos;
    float distToMove = SampleRadialCurve(direction.length());
    direction.normalize();
    m_LastPos = m_LastPos + (direction * distToMove);

    //Catch NaNs and interrupted input
    if ( !((std::isfinite(m_LastPos.x)) && (std::isfinite(m_LastPos.y))) || ((m_DetectInterruptions) && (::GetTickCount64() > m_LastTick + 50)) )
        m_LastPos = target;

    m_LastTick = ::GetTickCount64();

    return m_LastPos;
}

Vector3 RadialFollowCore::Filter(const Vector3& target)
{
    Vector3 direction = target - m_LastPos3;
    float distToMove = SampleRadialCurve(direction.length());
    direction.normalize();
    m_LastPos3 = m_LastPos3 + (direction * distToMove);

    //Catch NaNs and interrupted input
    if ( !((std::isfinite(m_LastPos3.x)) && (std::isfinite(m_LastPos3.y)) && (std::isfinite(m_LastPos3.z))) || ((m_DetectInterruptions) && (::GetTickCount64() > m_LastTick + 50)) )
        m_LastPos3 = target;

    m_LastTick = ::GetTickCount64();

    return m_LastPos3;
}

Vector3 RadialFollowCore::FilterWrapped(const Vector3& target, float value_min, float value_max)
{
    return FilterWrapped(target, value_min, value_max, Vector3((float)m_RadiusInner, (float)m_RadiusInner, (float)m_RadiusInner));
}

Vector3 RadialFollowCore::FilterWrapped(const Vector3& target, float value_min, float value_max, const Vector3& inner_radii)
{
    auto unwrap_to_nearest = [&](const float value_wrapped, const float value_prev)
    {
        //Unwrap value in a way that ensures close to the previous one
        const float range_width = value_max - value_min;
        const float value_in_range = value_wrapped - std::floor((value_wrapped - value_min) / range_width) * range_width;
        //Shift by multiples of range_width so the value is +/- half range_width to the previous value
        const float delta = value_prev - value_in_range;
        return value_in_range + std::round(delta / range_width) * range_width;
    };

    Vector3 target_unwrapped = target;

    target_unwrapped.x = unwrap_to_nearest(target.x, m_LastPos3.x);
    target_unwrapped.y = unwrap_to_nearest(target.y, m_LastPos3.y);
    target_unwrapped.z = unwrap_to_nearest(target.z, m_LastPos3.z);

    Vector3 direction = target_unwrapped - m_LastPos3;
    const float dist = direction.length();
    double radius_inner = 0.0;

    if (dist > 0.0f)
    {
        const Vector3 direction_normalized = direction * (1.0f / dist);
        double inverse_radius_squared = 0.0;

        const auto add_axis = [&](float direction_axis, float radius_axis)
        {
            if (std::abs(direction_axis) > 0.000001f)
            {
                if (radius_axis <= 0.0f)
                    inverse_radius_squared = INFINITY;
                else
                    inverse_radius_squared += (direction_axis * direction_axis) / (radius_axis * radius_axis);
            }
        };

        add_axis(direction_normalized.x, inner_radii.x);
        add_axis(direction_normalized.y, inner_radii.y);
        add_axis(direction_normalized.z, inner_radii.z);

        if ((inverse_radius_squared > 0.0) && (std::isfinite(inverse_radius_squared)))
            radius_inner = 1.0 / std::sqrt(inverse_radius_squared);
    }

    float distToMove = (float)DeltaFn(dist, m_XOffset, m_ScaleComp, radius_inner);
    direction.normalize();
    m_LastPos3 = m_LastPos3 + (direction * distToMove);

    //Catch NaNs and interrupted input
    if ( !((std::isfinite(m_LastPos3.x)) && (std::isfinite(m_LastPos3.y)) && (std::isfinite(m_LastPos3.z))) || ((m_DetectInterruptions) && (::GetTickCount64() > m_LastTick + 50)) )
        m_LastPos3 = target;

    m_LastTick = ::GetTickCount64();

    return m_LastPos3;
}

void RadialFollowCore::ResetLastPos()
{
    //Filter() functions will already fall back to the target pos if any results aren't finite so this does the trick
    m_LastPos.x  = INFINITY;
    m_LastPos3.x = INFINITY;
}

void RadialFollowCore::UpdateDerivedParams()
{
    if (m_SoftKneeScale > 0.0001f)
    {
        m_XOffset   = GetXOffset();
        m_ScaleComp = GetScaleComp();
    }
    else //Calculating them with functions would use / by 0
    {
        m_XOffset   = -1.0;
        m_ScaleComp =  1.0;
    }
}

double RadialFollowCore::KneeFunc(double x)
{
    if (x < -3.0)
        return x;
    else if (x < 3.0)
        return log(tanh(exp(x)));
    else
        return 0.0;
}

double RadialFollowCore::KneeScaled(double x)
{
    if (m_SoftKneeScale > 0.0001)
        return m_SoftKneeScale * KneeFunc(x / m_SoftKneeScale) + 1.0;
    else
        return (x > 0.0) ? 1.0 : 1.0 + x;
}

double RadialFollowCore::InverseTanh(double x)
{
    return log((1.0 + x) / (1.0 - x)) / 2.0;
}

double RadialFollowCore::InverseKneeScaled(double x)
{
    return m_SoftKneeScale * log(InverseTanh(exp((x - 1.0) / m_SoftKneeScale)));
}

double RadialFollowCore::DeriveKneeScaled(double x)
{
    const double x_e = exp(x / m_SoftKneeScale);
    const double x_e_tanh = tanh(x_e);
    return (x_e - x_e * (x_e_tanh * x_e_tanh)) / x_e_tanh;
}

double RadialFollowCore::GetXOffset()
{
    return InverseKneeScaled(0.0);
}

double RadialFollowCore::GetScaleComp()
{
    return DeriveKneeScaled(GetXOffset());
}

double RadialFollowCore::GetRadiusOuterAdjusted(double radius_inner)
{
    return m_GridScale * std::max(m_RadiusOuter, radius_inner + 0.0001);
}

double RadialFollowCore::GetRadiusInnerAdjusted(double radius_inner)
{
    return m_GridScale * radius_inner;
}

double RadialFollowCore::LeakedFn(double x, double offset, double scaleComp)
{
    return KneeScaled(x + offset) * (1 - m_SmoothingLeakCoef) + x * m_SmoothingLeakCoef * scaleComp;
}

double RadialFollowCore::SmoothedFn(double x, double offset, double scaleComp)
{
    return LeakedFn(x * m_SmoothingCoef / scaleComp, offset, scaleComp);
}

double RadialFollowCore::ScaleToOuter(double x, double offset, double scaleComp, double radius_inner)
{
    return (GetRadiusOuterAdjusted(radius_inner) - GetRadiusInnerAdjusted(radius_inner)) *
           SmoothedFn(x / (GetRadiusOuterAdjusted(radius_inner) - GetRadiusInnerAdjusted(radius_inner)), offset, scaleComp);
}

double RadialFollowCore::DeltaFn(double x, double offset, double scaleComp, double radius_inner)
{
    return (x > GetRadiusInnerAdjusted(radius_inner)) ?
           x - ScaleToOuter(x - GetRadiusInnerAdjusted(radius_inner), offset, scaleComp, radius_inner) - GetRadiusInnerAdjusted(radius_inner) : 0.0;
}
