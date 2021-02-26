#pragma once

enum class CustomMsgTypes : uint32_t
{
    ServerAccept,
    ServerOutput,
    BackToHomePosition,
    MoveToObstacleCheckingPosition,
    MoveToInitVisionPosition,
    MoveToCleanTaskInitPosition,
};

