/*
 This file is part of SpatGRIS.

 Developers: Gaël Lane Lépine, Samuel Béland, Olivier Bélanger, Nicolas Masson

 SpatGRIS is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 SpatGRIS is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with SpatGRIS.  If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#include "StrongTypes/sg_CartesianVector.hpp"
#include "StrongTypes/sg_Dbfs.hpp"
#include "StrongTypes/sg_Hz.hpp"
#include "StrongTypes/sg_Radians.hpp"
#include "StrongTypes/sg_SourceIndex.hpp"
#include "Containers/sg_AtomicUpdater.hpp"
#include "Containers/sg_StrongArray.hpp"
#include "Data/StrongTypes/sg_OutputPatch.hpp"
#include "Data/sg_LogicStrucs.hpp"
#include "Data/sg_SpatMode.hpp"
#include "Data/sg_constants.hpp"
#include "sg_AudioStructs.hpp"
#include "sg_Position.hpp"
#include "sg_Triplet.hpp"
#include "juce_core/juce_core.h"
#include "juce_graphics/juce_graphics.h"
#include "tl/optional.hpp"
#include <atomic>
#include <cstdint>
#include <memory>
#include "../Containers/sg_OwnedMap.hpp"
#include "../Containers/sg_StaticMap.hpp"

// this is the main switch to enable/disable fork_union
#define SG_USE_FORK_UNION 0

// fork_union concurrency method options (used to set SG_FU_METHOD right below)
// Use an array of atomics for concurrent float access: (vector<vector<AtomicWrapper<float>>>)
#define SG_FU_USE_ARRAY_OF_ATOMICS 1

// Use a buffer per thread for concurrent float access: (vector<vector<vector<float>>>)
#define SG_FU_USE_BUFFER_PER_THREAD 2

// Use std::atomic_ref<float> for lock-free atomic access to floats
#define SG_FU_USE_ATOMIC_CAST 3

// and SG_FU_METHOD is the "algorithm" used by fork_union, which is set to one of the above macros
#define SG_FU_METHOD SG_FU_USE_ATOMIC_CAST

namespace gris
{
constexpr auto DEFAULT_UDP_OUTPUT_PORT = 18022;
constexpr auto DEFAULT_UDP_INPUT_PORT = 18023;

constexpr auto DEFAULT_OSC_INPUT_PORT = 18032;
constexpr auto MAX_OSC_INPUT_PORT = 65535;

//==============================================================================
// TODO: rename SliceState to SpeakerIOState to better reflect its purpose
/** I/O state of a speaker, whether it's muted, soloed, or just normal */
enum class SliceState : std::uint8_t { normal, muted, solo };

// TODO: rename sliceStateToString() to IOStateToString() to better reflect its purpose
[[nodiscard]] juce::String sliceStateToString(SliceState state);

// TODO: rename stringToSliceState() to stringToIOState() to better reflect its purpose
[[nodiscard]] tl::optional<SliceState> stringToSliceState(juce::String const & string);

//==============================================================================
/** Attenuation Bypass State. Invalid is here to keep project file compatiblity */
enum class AttenuationBypassSate : std::uint8_t { invalid, on, off };
[[nodiscard]] juce::String attenuationBypassStateToString(AttenuationBypassSate state);
[[nodiscard]] AttenuationBypassSate stringToAttenuationBypassState(juce::String const & string);

#if SG_USE_FORK_UNION
    #if SG_FU_METHOD == SG_FU_USE_ARRAY_OF_ATOMICS
/* Taken from https://stackoverflow.com/questions/13193484/how-to-declare-a-vector-of-atomic-in-c
 **/
template<typename T>
struct AtomicWrapper {
    std::atomic<T> _a;

    AtomicWrapper() : _a() {}

    AtomicWrapper(const std::atomic<T> & a) : _a(a.load()) {}

    AtomicWrapper(const AtomicWrapper & other) : _a(other._a.load()) {}

    /* This isn't atomic so shouldn't be done in concurent contexts! */
    AtomicWrapper & operator=(const AtomicWrapper & other) { _a.store(other._a.load()); }
};

// TODO FU: instead of using another set of arrays, could we deal with concurrency inside SpeakerAudioBuffer directly?
// TODO FU: explore using a boost::multi_array or https://github.com/correaa/boost-multi, with the important point that
// we need to make sure buffers are separated by std::hardware_destructive_interference_size
using ForkUnionBuffer = std::vector<std::vector<AtomicWrapper<float>>>;

    #elif SG_FU_METHOD == SG_FU_USE_BUFFER_PER_THREAD
using ForkUnionBuffer = std::vector<std::vector<std::vector<float>>>;

    #elif SG_FU_METHOD == SG_FU_USE_ATOMIC_CAST
static_assert(std::atomic_ref<float>::is_always_lock_free, "float cannot be converted to a lock-free atomic_ref!");
    #endif
#endif

//==============================================================================
/** For the following data structures, we use the following semantics:
 *
 * COLD  : no concurrent access. All reads and writes happen on the same thread.
 * WARM  : concurrent access, but never during a performance. Usually done with a lock.
 * HOT   : concurrent access DURING PERFORMANCE. Usually done with an AtomicUpdater<>.
 * MIXED : a top-level structure that holds multiple access patterns.
 *
 * Note that the term "Config" is associated with WARM data transmitted from the message thread to another thread and
 * that the term "State" is associate with COLD data created and accessed by something else that the message thread.
 */

//==============================================================================
/** The settings associated with the 2D and 3D viewports.
 *
 * COLD
 */
struct ViewSettings {
    bool keepSpeakerViewWindowOnTop{ false };
    bool showHall{ false };
    bool showSpeakers{ true };
    bool showSourceNumbers{ false };
    bool showSpeakerNumbers{ false };
    bool showSpeakerTriplets{ false };
    bool showSpeakerLevels{ false };
    bool showSphereOrCube{ false };
    bool showSourceActivity{ false };
    //==============================================================================
    [[nodiscard]] std::unique_ptr<juce::XmlElement> toXml() const;
    [[nodiscard]] static tl::optional<ViewSettings> fromXml(juce::XmlElement const & xml);
    //==============================================================================
    struct XmlTags {
        static juce::String const MAIN_TAG;
        static juce::String const KEEP_SPEAKERVIEW_ON_TOP;
        static juce::String const SHOW_HALL;
        static juce::String const SHOW_SPEAKERS;
        static juce::String const SHOW_SOURCE_NUMBERS;
        static juce::String const SHOW_SPEAKER_NUMBERS;
        static juce::String const SHOW_SPEAKER_TRIPLETS;
        static juce::String const SHOW_SPEAKER_LEVELS;
        static juce::String const SHOW_SPHERE_OR_CUBE;
        static juce::String const SHOW_SOURCE_ACTIVITY;
    };
};

//==============================================================================
/** The data needed to display a source in the 3D viewport.
 *
 * HOT
 */
struct ViewportSourceData {
    Position position{};
    /** Between 0 and 1. */
    float azimuthSpan{};
    /** Between 0 and 1. */
    float zenithSpan{};
    juce::Colour colour{};
    /** Only used in hybrid mode. */
    SpatMode hybridSpatMode{};
};

/** The updater type used to send a source's data from the message thread to the OpenGL thread. */
using ViewportSourceDataUpdater = AtomicUpdater<tl::optional<ViewportSourceData>>;

//==============================================================================
/** The data needed to display a speaker in the 3D viewport.
 *
 * WARM
 */
struct ViewportSpeakerConfig {
    Position position{};
    bool isSelected{};
    bool isDirectOutOnly{};
};

/** The updater type used to send a speaker's alpha level from the message thread to the OpenGL thread. */
using ViewportSpeakerAlphaUpdater = AtomicUpdater<float>;

//==============================================================================
/** The data needed by the 3D viewport.
 *
 * WARM
 */
struct ViewportConfig {
    StaticMap<output_patch_t, ViewportSpeakerConfig, MAX_NUM_SPEAKERS> speakers{};
    ViewSettings viewSettings{};
    SpatMode spatMode{};
    juce::String title{};
};

//==============================================================================
/** The inner (no concurrent access) 3D viewport data.
 *
 * COLD
 */
struct ViewportState {
    StrongArray<source_index_t, ViewportSourceDataUpdater::Token *, MAX_NUM_SOURCES> mostRecentSourcesData{};
    StrongArray<output_patch_t, ViewportSpeakerAlphaUpdater::Token *, MAX_NUM_SPEAKERS> mostRecentSpeakersAlpha{};
    float cameraZoomVelocity{};
    Position cameraPosition{};
    juce::int64 lastRenderTimeMs{ juce::Time::currentTimeMillis() };
    juce::Point<float> rayClick{};
    bool shouldRayCast{};
    juce::Point<float> panMouseOrigin{};
    PolarVector panCameraOrigin{};
    float displayScaling{};
    juce::Array<Triplet> triplets{}; // TODO : this should be part of the ViewportConfig?
};

//==============================================================================
/** All of the 3D viewport's data.
 *
 * MIXED
 */
struct ViewportData {
    ViewportConfig warmData{};
    ViewportState coldData{};
    StaticMap<source_index_t, ViewportSourceDataUpdater, MAX_NUM_SOURCES> hotSourcesDataUpdaters{};
    StrongArray<output_patch_t, ViewportSpeakerAlphaUpdater, MAX_NUM_SPEAKERS> hotSpeakersAlphaUpdaters{};
};

//==============================================================================
/** All of the data associated with a source.
 *
 * COLD
 */
struct SourceData {
    SliceState state{};                // normal / muted / solo
    tl::optional<Position> position{}; // tl::nullopt if the source is inactive
    float azimuthSpan{};
    float zenithSpan{};
    tl::optional<output_patch_t> directOut{};
    juce::Colour colour{ DEFAULT_SOURCE_COLOR };
    SpatMode hybridSpatMode{};
    //==============================================================================
    [[nodiscard]] SourceAudioConfig toConfig(bool soloMode) const;
    [[nodiscard]] ViewportSourceData toViewportData(float alpha) const;
    [[nodiscard]] std::unique_ptr<juce::XmlElement> toXml(source_index_t index) const;
    [[nodiscard]] static tl::optional<SourceData> fromXml(juce::XmlElement const & xml);
    [[nodiscard]] bool operator==(SourceData const & other) const noexcept;
    //==============================================================================
    struct XmlTags {
        static juce::String const STATE;
        static juce::String const DIRECT_OUT;
        static juce::String const COLOUR;
        static juce::String const HYBRID_SPAT_MODE;
        static juce::String const MAIN_TAG_PREFIX;
    };
};

//==============================================================================
using SourcesData = OwnedMap<source_index_t, SourceData, MAX_NUM_SOURCES>;

//==============================================================================
struct SpeakerHighpassData {
    hz_t freq{};
    //==============================================================================
    [[nodiscard]] SpeakerHighpassConfig toConfig(double sampleRate) const;
    [[nodiscard]] std::unique_ptr<juce::XmlElement> toXml() const;
    [[nodiscard]] static tl::optional<SpeakerHighpassData> fromXml(juce::XmlElement const & xml);
    [[nodiscard]] static tl::optional<SpeakerHighpassData> fromVt(juce::ValueTree const & vt) noexcept;
    [[nodiscard]] bool operator==(SpeakerHighpassData const & other) const noexcept;
    //==============================================================================
    struct XmlTags {
        static juce::String const HIGHPASS;
        static juce::String const FREQ;
    };
};

//==============================================================================
struct SpeakerData {
    SliceState state{};
    Position position{ PolarVector{ radians_t{ 0.0f }, radians_t{ 0.0f }, 1.0f } };
    dbfs_t gain{};
    tl::optional<SpeakerHighpassData> highpassData{};
    float peak{};
    bool isSelected{};
    bool isDirectOutOnly{};
    //==============================================================================
    [[nodiscard]] SpeakerAudioConfig toConfig(bool soloMode, double sampleRate) const noexcept;
    [[nodiscard]] ViewportSpeakerConfig toViewportConfig() const noexcept;
    [[nodiscard]] std::unique_ptr<juce::XmlElement> toXml(output_patch_t outputPatch) const noexcept;
    [[nodiscard]] juce::ValueTree toVt(output_patch_t outputPatch) const noexcept;
    [[nodiscard]] static tl::optional<SpeakerData> fromXml(juce::XmlElement const & xml) noexcept;
    [[nodiscard]] static tl::optional<SpeakerData> fromVt(juce::ValueTree vt) noexcept;

    /**
     * @brief Returns the absolute position of a speaker from a given ValueTree.
     *
     * This function extracts and returns the absolute position of a speaker with regards to the origin,
     * even if the speaker is inside a group, from the provided ValueTree.
     *
     * @param vt The ValueTree containing the speaker data.
     * @return tl::optional<Position> The absolute position if available, otherwise tl::nullopt.
     */
    static tl::optional<Position> getAbsoluteSpeakerPosition(juce::ValueTree vt);

    using Quaternion = std::array<float, 4>;
    /**
     * @brief Computes the speaker's absolute position given a speaker's position relative to its parent, its parent's
     * position and its parent's rotation quaternion. This rotates the local position and calls the overload which adds
     * the groups position to the relative position of the speaker.

     *
     * @param localSpeakerPosition The position of the speaker relative to its parent.
     * @param parentPosition The absolute position of the parent.
     * @param parentQuat The rotation quaternion of the parent group
     * @return tl::optional<Position> The absolute position of the speaker if calculation is possible, otherwise
     * tl::nullopt.
     */
    static tl::optional<Position> getAbsoluteSpeakerPosition(Position localSpeakerPosition,
                                                             Position parentPosition,
                                                             tl::optional<Quaternion> parentQuat);

    [[nodiscard]] bool operator==(SpeakerData const & other) const noexcept;
    //==============================================================================
    struct XmlTags {
        static juce::String const STATE;
        static juce::String const GAIN;
        static juce::String const IS_DIRECT_OUT_ONLY;
        static juce::String const MAIN_TAG_PREFIX;
    };
};

//==============================================================================
using SpeakersData = OwnedMap<output_patch_t, SpeakerData, MAX_NUM_SPEAKERS>;

//==============================================================================
struct MbapDistanceAttenuationData {
    hz_t freq{ 16000.0f };
    dbfs_t attenuation{};
    AttenuationBypassSate attenuationBypassState{};
    //==============================================================================
    [[nodiscard]] MbapAttenuationConfig toConfig(double sampleRate, bool shouldProcess) const;
    [[nodiscard]] std::unique_ptr<juce::XmlElement> toXml() const;
    [[nodiscard]] static tl::optional<MbapDistanceAttenuationData> fromXml(juce::XmlElement const & xml);
    [[nodiscard]] bool operator==(MbapDistanceAttenuationData const & other) const noexcept;
    //==============================================================================
    struct XmlTags {
        static juce::String const MAIN_TAG;
        static juce::String const LEGACY_MAIN_TAG;
        static juce::String const FREQ;
        static juce::String const ATTENUATION;
        static juce::String const BYPASS;
    };
};

//==============================================================================
struct AudioSettings {
    juce::String deviceType{};
    juce::String inputDevice{};
    juce::String outputDevice{};
    double sampleRate{ DEFAULT_SAMPLE_RATE };
    int bufferSize{ DEFAULT_BUFFER_SIZE };
    //==============================================================================
    [[nodiscard]] std::unique_ptr<juce::XmlElement> toXml() const;
    [[nodiscard]] static tl::optional<AudioSettings> fromXml(juce::XmlElement const & xml);
    //==============================================================================
    struct XmlTags {
        static juce::String const MAIN_TAG;
        static juce::String const INTERFACE_TYPE;
        static juce::String const INPUT_INTERFACE;
        static juce::String const OUTPUT_INTERFACE;
        static juce::String const SAMPLE_RATE;
        static juce::String const BUFFER_SIZE;
    };
};

//==============================================================================
enum class RecordingFormat : std::uint8_t {
    wav,
    aiff
#ifdef USE_CAF
    ,
    caf
#endif
};
constexpr auto DEFAULT_RECORDING_FORMAT{ RecordingFormat::wav };
enum class RecordingFileType : std::uint8_t { mono, interleaved };
constexpr auto DEFAULT_RECORDING_FILE_TYPE{ RecordingFileType::mono };

juce::String recordingFormatToString(RecordingFormat format);
tl::optional<RecordingFormat> stringToRecordingFormat(juce::String const & string);
juce::String recordingFileTypeToString(RecordingFileType fileType);
tl::optional<RecordingFileType> stringToRecordingFileType(juce::String const & string);

//==============================================================================
struct RecordingOptions {
    RecordingFormat format{ DEFAULT_RECORDING_FORMAT };
    RecordingFileType fileType{ DEFAULT_RECORDING_FILE_TYPE };
    bool shouldSaveSpeakerSetup{};
    //==============================================================================
    [[nodiscard]] std::unique_ptr<juce::XmlElement> toXml() const;
    [[nodiscard]] static tl::optional<RecordingOptions> fromXml(juce::XmlElement const & xml);
    //==============================================================================
    struct XmlTags {
        static juce::String const MAIN_TAG;
        static juce::String const FORMAT;
        static juce::String const FILE_TYPE;
    };
};

//==============================================================================
struct StereoRouting {
    output_patch_t left{ 1 };
    output_patch_t right{ 2 };
    //==============================================================================
    [[nodiscard]] std::unique_ptr<juce::XmlElement> toXml() const;
    [[nodiscard]] static tl::optional<StereoRouting> fromXml(juce::XmlElement const & xml);
    //==============================================================================
    struct XmlTags {
        static juce::String const MAIN_TAG;
        static juce::String const LEFT;
        static juce::String const RIGHT;
    };
};

//==============================================================================
using SourcesOrdering = juce::Array<source_index_t>;

//==============================================================================
struct ProjectData {
    SourcesData sources{};
    SourcesOrdering ordering{};
    MbapDistanceAttenuationData mbapDistanceAttenuationData{};
    int oscPort{ DEFAULT_OSC_INPUT_PORT };
    tl::optional<int> standaloneSpeakerViewInputPort;
    tl::optional<int> standaloneSpeakerViewOutputPort;
    tl::optional<juce::String> standaloneSpeakerViewOutputAddress;
    dbfs_t masterGain{};
    float spatGainsInterpolation{};
    SpatMode spatMode{};
    //==============================================================================
    [[nodiscard]] std::unique_ptr<juce::XmlElement> toXml() const;
    [[nodiscard]] static tl::optional<ProjectData> fromXml(juce::XmlElement const & xml);
    [[nodiscard]] bool operator==(ProjectData const & other) const noexcept;
    [[nodiscard]] bool operator!=(ProjectData const & other) const noexcept { return !(*this == other); }
    //==============================================================================
    struct XmlTags {
        static juce::String const MAIN_TAG;
        static juce::String const VERSION;
        static juce::String const SPAT_MODE;
        static juce::String const SOURCES;
        static juce::String const MASTER_GAIN;
        static juce::String const GAIN_INTERPOLATION;
        static juce::String const OSC_PORT;
        static juce::String const STANDALONE_SPEAKERVIEW_INPUT_PORT;
        static juce::String const STANDALONE_SPEAKERVIEW_OUTPUT_PORT;
        static juce::String const STANDALONE_SPEAKERVIEW_OUTPUT_ADDRESS;
    };
};

//==============================================================================
struct AppData {
    AudioSettings audioSettings{};
    RecordingOptions recordingOptions{};
    ViewSettings viewSettings{};
    CartesianVector cameraPosition{ -0.5256794095039368f, -2.008379459381104f, 1.312143206596375f };
    juce::Point<int> speakerViewWindowPosition{};
    juce::Point<int> speakerViewWindowSize{};

    /** The full path to the last loaded speaker setup file. */
    juce::String lastSpeakerSetup{};
    juce::String lastProject{};
    juce::String lastRecordingDirectory{
        juce::File::getSpecialLocation(juce::File::SpecialLocationType::userDesktopDirectory).getFullPathName()
    };
    tl::optional<StereoMode> stereoMode{};
    StereoRouting stereoRouting{};
    bool playerExists{};
    int windowX{ 100 };
    int windowY{ 100 };
    int windowWidth{ 1200 };
    int windowHeight{ 637 };
    double sashPosition{ -0.4694048616932104 }; // TODO: should be removed since SpeakerView
    //==============================================================================
    [[nodiscard]] std::unique_ptr<juce::XmlElement> toXml() const;
    [[nodiscard]] static tl::optional<AppData> fromXml(juce::XmlElement const & xml);
    //==============================================================================
    struct XmlTags {
        static juce::String const MAIN_TAG;
        static juce::String const LAST_SPEAKER_SETUP;
        static juce::String const LAST_PROJECT;
        static juce::String const LAST_RECORDING_DIRECTORY;
        static juce::String const LAST_STEREO_MODE;
        static juce::String const WINDOW_X;
        static juce::String const WINDOW_Y;
        static juce::String const WINDOW_WIDTH;
        static juce::String const WINDOW_HEIGHT;
        static juce::String const SV_WINDOW_X;
        static juce::String const SV_WINDOW_Y;
        static juce::String const SV_WINDOW_WIDTH;
        static juce::String const SV_WINDOW_HEIGHT;
        static juce::String const SASH_POSITION;
        static juce::String const CAMERA;
    };
};

//==============================================================================
using SpeakersOrdering = juce::Array<output_patch_t>;

//==============================================================================
struct SpeakerSetup {
    // for now this is doubled data, but in time it will replace/supplement the rest of the data in here
    juce::ValueTree speakerSetupValueTree;

    SpeakersData speakers{};
    SpeakersOrdering ordering{};
    SpatMode spatMode{};
    float diffusion{};
    bool generalMute{};
    //==============================================================================
    [[nodiscard]] std::unique_ptr<juce::XmlElement> toXml() const;
    [[nodiscard]] static tl::optional<SpeakerSetup> fromXml(juce::XmlElement const & xml);
    [[nodiscard]] bool operator==(SpeakerSetup const & other) const noexcept;
    [[nodiscard]] bool operator!=(SpeakerSetup const & other) const noexcept { return !(*this == other); }
    [[nodiscard]] bool isDomeLike() const noexcept;
    [[nodiscard]] SpeakersAudioConfig toAudioConfig(double sampleRate) const noexcept;
    [[nodiscard]] int numOfSpatializedSpeakers() const noexcept;
    [[nodiscard]] static juce::ValueTree toVt(const SpeakerSetup & legacySpeakerSetup);
    //==============================================================================
    struct XmlTags {
        static juce::String const MAIN_TAG;
        static juce::String const VERSION;
        static juce::String const SPAT_MODE;
        static juce::String const DIFFUSION;
        static juce::String const GENERAL_MUTE;
    };
};

//==============================================================================
struct SpatGrisData {
    SpeakerSetup speakerSetup{};
    ProjectData project{};
    AppData appData{};
    tl::optional<dbfs_t> pinkNoiseLevel{};
    AtomicUpdater<SourcePeaks>::Token * mostRecentSourcePeaks{};
    AtomicUpdater<SpeakerPeaks>::Token * mostRecentSpeakerPeaks{};
    AtomicUpdater<StereoPeaks>::Token * mostRecentStereoPeaks{};
    //==============================================================================
    [[nodiscard]] std::unique_ptr<AudioConfig> toAudioConfig() const;
    [[nodiscard]] ViewportConfig toViewportConfig() const noexcept;
};
} // namespace gris
