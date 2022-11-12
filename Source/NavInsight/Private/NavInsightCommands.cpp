// Copyright Epic Games, Inc. All Rights Reserved.

#include "NavInsightCommands.h"

#define LOCTEXT_NAMESPACE "FNavInsightModule"

void FNavInsightCommands::RegisterCommands()
{
	UI_COMMAND(PluginAction, "NavInsight", "Execute NavInsight action", EUserInterfaceActionType::Button, FInputGesture());
}

#undef LOCTEXT_NAMESPACE
