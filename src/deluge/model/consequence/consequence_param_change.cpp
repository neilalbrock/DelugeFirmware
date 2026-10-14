/*
 * Copyright © 2017-2023 Synthstrom Audible Limited
 *
 * This file is part of The Synthstrom Audible Deluge Firmware.
 *
 * The Synthstrom Audible Deluge Firmware is free software: you can redistribute it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with this program.
 * If not, see <https://www.gnu.org/licenses/>.
 */

#include "model/consequence/consequence_param_change.h"
#include "definitions_cxx.hpp"
#include "model/clip/instrument_clip.h"
#include "model/consequence/consequence.h"
#include "model/note/note_row.h"
#include "modulation/params/param_collection.h"
#include "modulation/params/param_node_vector.h"
#include "util/functions.h"
#include <new>

ConsequenceParamChange::ConsequenceParamChange(ModelStackWithAutoParam const* modelStack, bool stealData) {
	type = Consequence::PARAM_CHANGE;
	memcpy(modelStackMemory, modelStack, sizeof(ModelStackWithParamId));

	NoteRow* noteRow = modelStack->getNoteRowAllowNull();
	paramManagerIsNoteRows = (noteRow && modelStack->paramManager == &noteRow->paramManager);
	summaryIndex = paramManagerIsNoteRows ? (modelStack->summary - modelStack->paramManager->summaries) : 0;

	state.value = modelStack->autoParam->currentValue;

	// Either steal the data...
	if (stealData) {
		state.nodes.swapStateWith(&modelStack->autoParam->nodes);
		snapshotTaken = true;
	}

	// Or clone it...
	else {
		snapshotTaken = state.nodes.cloneFrom(&modelStack->autoParam->nodes);
	}
}

Error ConsequenceParamChange::revert(TimeType time, ModelStack* modelStackWithSong) {

	// We only actually store one state at a time - either the before, or the after. As we revert in either direction,
	// we swap our stored state with that of the param in question - like, actually swap the pointer to the
	// ParamNodeVector, so it's real efficient!

	if (paramManagerIsNoteRows) {
		// The NoteRow (with its ParamManager and summaries inside it) may have moved since we were made, so find it
		// again by id
		TimelineCounter* timelineCounter = modelStack.getTimelineCounterAllowNull();
		NoteRow* noteRow =
		    timelineCounter ? ((InstrumentClip*)timelineCounter)->getNoteRowFromId(modelStack.noteRowId) : nullptr;
		if (!noteRow) {
			return Error::BUG;
		}
		ParamCollectionSummary* summary = &noteRow->paramManager.summaries[summaryIndex];
		if (summary->paramCollection != modelStack.paramCollection) {
			return Error::BUG; // Whatever has that id now isn't the NoteRow this was recorded for
		}
		modelStack.setNoteRow(noteRow);
		modelStack.paramManager = &noteRow->paramManager;
		modelStack.summary = summary;
	}

	modelStack.paramCollection->remotelySwapParamState(&state, &modelStack);

	return Error::NONE;
}
