// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Framework/Commands/Commands.h"
#include "NavInsightStyle.h"

class FNavInsightCommands : public TCommands<FNavInsightCommands>
{
public:

	FNavInsightCommands()
		: TCommands<FNavInsightCommands>(TEXT("NavInsight"), NSLOCTEXT("Contexts", "NavInsight", "NavInsight Plugin"), NAME_None, FNavInsightStyle::GetStyleSetName())
	{
	}

	// TCommands<> interface
	virtual void RegisterCommands() override;

public:
	TSharedPtr< FUICommandInfo > PluginAction;
};
